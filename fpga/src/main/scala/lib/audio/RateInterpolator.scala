package lib.audio

import chisel3._
import chisel3.util._

/**
 * Fractional-ratio resampler: consumes a stereo stream at the intermediate rate and produces
 * one output sample per `outEnable` pulse using Catmull-Rom cubic interpolation over a
 * 4-sample window.
 *
 * The input/output rate ratio is the exact rational phaseNum / phaseDen: the phase
 * accumulator adds phaseNum per output sample and subtracts phaseDen per consumed input
 * sample. When both rates derive from a common clock the ratio is exact and the ring buffer
 * occupancy stays constant, with no rate control loop. If arrivals and consumption ever
 * diverge (samples dropped upstream, outEnable stalled), the interpolator flushes and
 * re-locks, holding the last output while the ring re-prefills.
 *
 * Input samples must already be band-limited to below half the output rate; the cubic
 * kernel suppresses the interpolation images of the input rate by ~55 dB or more in the
 * audio band at ~4x oversampling.
 *
 * Interpolation runs serially over a few cycles after each outEnable pulse, asserting
 * outValid when the result is ready.
 */

class RateInterpolator(
  dataBits: Int,
  phaseNum: Int,
  phaseDen: Int,
) extends Module {
  require(phaseNum > 2 * phaseDen, "expects an oversampled input (ratio > 2)")
  require(phaseDen >= 1 && (phaseDen - 1) * math.round(65536.0 / phaseDen) < 65536,
    "phaseDen too large for Q0.16 fraction")

  private val ringBits = 4
  private val ringDepth = 1 << ringBits
  // Entries the consume guard keeps resident for the interpolation window s0..s3.
  private val windowGuard = 4
  private val ceilRatio = (phaseNum + phaseDen - 1) / phaseDen
  // Steady-state occupancy: clears the window guard by more than the consume pattern's
  // swing, with margin below the ring capacity for arrival jitter.
  private val prefill = windowGuard + ceilRatio + 2
  require(prefill + ceilRatio < ringDepth, "ring too small for prefill plus consume swing")
  // Q0.16 fraction = acc * tRecip, computed with an elaboration-time reciprocal.
  private val tRecip = math.round(65536.0 / phaseDen).toInt
  private val accBits = log2Ceil(phaseNum + phaseDen) + 1

  /** Output has one extra bit: the cubic kernel can overshoot by up to ~7%. */
  val outputBits = dataBits + 1

  val io = IO(new Bundle {
    /** Input stream at the intermediate rate. */
    val inValid = Input(Bool())
    val inLeft = Input(SInt(dataBits.W))
    val inRight = Input(SInt(dataBits.W))

    /** Pulsed once per output sample period (e.g. the I2S transmitter's sampleEnable). */
    val outEnable = Input(Bool())
    /** Pulses when a newly interpolated sample is available on outLeft/outRight. */
    val outValid = Output(Bool())
    val outLeft = Output(SInt(outputBits.W))
    val outRight = Output(SInt(outputBits.W))
  })

  private val channels = 2
  val rings = Seq.fill(channels)(Mem(ringDepth, SInt(dataBits.W)))
  private val ins = Seq(io.inLeft, io.inRight)

  val writePtr = RegInit(0.U(ringBits.W))
  // Points at s0, the oldest sample of the interpolation window s0..s3.
  val readPtr = RegInit(0.U(ringBits.W))
  // Counted explicitly: a pointer difference cannot distinguish empty from full.
  val occupancy = RegInit(0.U((ringBits + 1).W))
  val started = RegInit(false.B)

  val doWrite = io.inValid && occupancy < ringDepth.U
  val overrun = io.inValid && occupancy === ringDepth.U
  when (doWrite) {
    for (ch <- 0 until channels) {
      rings(ch).write(writePtr, ins(ch))
    }
    writePtr := writePtr + 1.U
  }

  val acc = RegInit(0.U(accBits.W))
  val tFrac = RegInit(0.U(16.W))
  val window = Seq.fill(channels)(Reg(Vec(4, SInt(dataBits.W))))

  val sIdle :: sConsume :: sLoad :: sMul :: sOutput :: Nil = Enum(5)
  val state = RegInit(sIdle)
  val step = RegInit(0.U(2.W))
  // Wide enough for the Horner intermediates: coefficients reach ~12x the input range.
  val horner = Seq.fill(channels)(RegInit(0.S((dataBits + 6).W)))

  val outReg = RegInit(VecInit(Seq.fill(channels)(0.S(outputBits.W))))
  val outValidReg = RegInit(false.B)
  outValidReg := false.B

  // Catmull-Rom polynomial coefficients for the window (t between s1 and s2), in Horner
  // order (innermost first): out = s1 + (t/2) * (c1 + t * (c2 + t * c3))
  private def catmullCoeffs(s: Vec[SInt]): Seq[SInt] = {
    val c1 = s(2) -& s(0)
    val c2 = ((s(0) << 1).asSInt -& (s(1) << 2).asSInt) -& s(1) +& (s(2) << 2).asSInt -& s(3)
    val diff12 = s(1) -& s(2)
    val c3 = ((diff12 << 1).asSInt +& diff12) +& s(3) -& s(0)
    Seq(c3, c2, c1)
  }
  private val coeffs = window.map(w => VecInit(catmullCoeffs(w)))
  private val tSigned = tFrac.zext

  private def hornerStep(current: SInt, c: SInt): SInt = {
    val product = (current +& c) * tSigned
    (product >> 16)(dataBits + 5, 0).asSInt
  }

  val consumeFire = WireDefault(false.B)
  val underrun = WireDefault(false.B)

  switch (state) {
    is (sIdle) {
      // Arming happens on an outEnable edge, so consumption begins immediately and the
      // steady-state occupancy pins at the prefill level.
      when (io.outEnable && (started || occupancy >= prefill.U)) {
        started := true.B
        acc := acc + phaseNum.U
        state := sConsume
      }
    }
    is (sConsume) {
      // Advance the window one input sample per cycle; the occupancy guard keeps
      // s3 behind the write pointer.
      when (acc >= phaseDen.U && occupancy > windowGuard.U) {
        acc := acc - phaseDen.U
        readPtr := readPtr + 1.U
        consumeFire := true.B
      } .elsewhen (acc >= phaseDen.U) {
        underrun := true.B
      } .otherwise {
        tFrac := (acc * tRecip.U)(15, 0)
        step := 0.U
        state := sLoad
      }
    }
    is (sLoad) {
      for (ch <- 0 until channels) {
        window(ch)(step) := rings(ch).read(readPtr + step)
      }
      step := step + 1.U
      when (step === 3.U) {
        step := 0.U
        state := sMul
      }
    }
    is (sMul) {
      for (ch <- 0 until channels) {
        horner(ch) := hornerStep(Mux(step === 0.U, 0.S, horner(ch)), coeffs(ch)(step))
      }
      step := step + 1.U
      when (step === 2.U) {
        state := sOutput
      }
    }
    is (sOutput) {
      for (ch <- 0 until channels) {
        outReg(ch) := FirFilter.saturate(window(ch)(1) +& (horner(ch) >> 1), outputBits)
      }
      outValidReg := true.B
      state := sIdle
    }
  }

  occupancy := (occupancy +& doWrite.asUInt - consumeFire.asUInt)(ringBits, 0)

  // A divergence between arrival and consumption rates cannot self-correct (the phase
  // accumulator has no absolute reference), so flush and re-lock from a fresh prefill.
  when (underrun || overrun) {
    started := false.B
    acc := 0.U
    occupancy := 0.U
    readPtr := Mux(doWrite, writePtr + 1.U, writePtr)
    state := sIdle
  }

  io.outLeft := outReg(0)
  io.outRight := outReg(1)
  io.outValid := outValidReg
}
