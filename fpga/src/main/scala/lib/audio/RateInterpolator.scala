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
 * sample. When both rates derive from a common clock the ratio is exact, so the ring buffer
 * occupancy stays bounded with no rate control loop.
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

  private val ringBits = 4 // 16-entry ring buffer
  private val prefill = 6
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

  val ringLeft = Mem(1 << ringBits, SInt(dataBits.W))
  val ringRight = Mem(1 << ringBits, SInt(dataBits.W))
  val writePtr = RegInit(0.U(ringBits.W))
  when (io.inValid) {
    ringLeft.write(writePtr, io.inLeft)
    ringRight.write(writePtr, io.inRight)
    writePtr := writePtr + 1.U
  }

  // Points at s0, the oldest sample of the interpolation window s0..s3.
  val readPtr = RegInit(0.U(ringBits.W))
  val available = writePtr - readPtr
  val started = RegInit(false.B)
  when (!started && available >= prefill.U) {
    started := true.B
  }

  val acc = RegInit(0.U(accBits.W))
  val tFrac = RegInit(0.U(16.W))
  val window = Seq(Reg(Vec(4, SInt(dataBits.W))), Reg(Vec(4, SInt(dataBits.W))))

  val sIdle :: sConsume :: sLoad :: sMul :: sOutput :: Nil = Enum(5)
  val state = RegInit(sIdle)
  val step = RegInit(0.U(2.W))
  // Wide enough for the Horner intermediates: coefficients reach ~12x the input range.
  val horner = Seq.fill(2)(RegInit(0.S((dataBits + 6).W)))

  val outLeftReg = RegInit(0.S(outputBits.W))
  val outRightReg = RegInit(0.S(outputBits.W))
  val outValidReg = RegInit(false.B)
  outValidReg := false.B

  // Catmull-Rom polynomial coefficients for the window (t between s1 and s2):
  //   out = s1 + (t/2) * (c1 + t * (c2 + t * c3))
  private def catmullCoeffs(s: Vec[SInt]): (SInt, SInt, SInt) = {
    val c1 = s(2) -& s(0)
    val c2 = ((s(0) << 1).asSInt -& (s(1) << 2).asSInt) -& s(1) +& (s(2) << 2).asSInt -& s(3)
    val diff12 = s(1) -& s(2)
    val c3 = ((diff12 << 1).asSInt +& diff12) +& s(3) -& s(0)
    (c1, c2, c3)
  }
  private val coeffsL = catmullCoeffs(window(0))
  private val coeffsR = catmullCoeffs(window(1))
  private val tSigned = Cat(0.U(1.W), tFrac).asSInt

  private def hornerStep(current: SInt, c: SInt): SInt = {
    val product = (current +& c) * tSigned
    (product >> 16)(dataBits + 5, 0).asSInt
  }

  switch (state) {
    is (sIdle) {
      when (io.outEnable && started) {
        acc := acc + phaseNum.U
        state := sConsume
      }
    }
    is (sConsume) {
      // Advance the window one input sample per cycle; the occupancy guard keeps
      // s3 behind the write pointer.
      when (acc >= phaseDen.U && available > 4.U) {
        acc := acc - phaseDen.U
        readPtr := readPtr + 1.U
      } .otherwise {
        tFrac := (acc * tRecip.U)(15, 0)
        step := 0.U
        state := sLoad
      }
    }
    is (sLoad) {
      window(0)(step) := ringLeft.read(readPtr + step)
      window(1)(step) := ringRight.read(readPtr + step)
      step := step + 1.U
      when (step === 3.U) {
        step := 0.U
        state := sMul
      }
    }
    is (sMul) {
      // Horner evaluation, innermost coefficient first: c3, then c2, then c1.
      switch (step) {
        is (0.U) {
          horner(0) := hornerStep(0.S, coeffsL._3)
          horner(1) := hornerStep(0.S, coeffsR._3)
        }
        is (1.U) {
          horner(0) := hornerStep(horner(0), coeffsL._2)
          horner(1) := hornerStep(horner(1), coeffsR._2)
        }
        is (2.U) {
          horner(0) := hornerStep(horner(0), coeffsL._1)
          horner(1) := hornerStep(horner(1), coeffsR._1)
        }
      }
      step := step + 1.U
      when (step === 2.U) {
        state := sOutput
      }
    }
    is (sOutput) {
      val left = window(0)(1) +& (horner(0) >> 1).asSInt
      val right = window(1)(1) +& (horner(1) >> 1).asSInt
      outLeftReg := FirFilter.saturate(left, outputBits)
      outRightReg := FirFilter.saturate(right, outputBits)
      outValidReg := true.B
      state := sIdle
    }
  }

  io.outLeft := outLeftReg
  io.outRight := outRightReg
  io.outValid := outValidReg
}
