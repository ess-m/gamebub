package lib.audio

import chisel3._
import chisel3.util._
import xilinx.{XpmCdcSingle, XpmFifoAsync}

/**
 * Configuration for [[AudioResampler]].
 *
 * All clocks on the handheld derive from the same MMCM VCO (50 MHz / 3 * 56.375), so every
 * rate ratio in the pipeline is an exact rational of the configured dividers -- the VCO
 * frequency cancels. This makes the final fractional resampling stage drift-free with a
 * pure integer phase accumulator.
 *
 * @param sysClockDivider MMCM divider producing the emulator core clock
 *                        (112 for the Game Boy core, 56 for GBA and boot)
 * @param avClockDivider  MMCM divider producing the audio/video (DPI) clock
 *                        (`revision.dpiClockDivider`)
 * @param audioMclkFactor AV clock cycles per DAC sample (`revision.audioMclkFactor`)
 */

case class AudioResamplerParams(
  sysClockDivider: Int,
  avClockDivider: Int,
  audioMclkFactor: Int,
) {
  val vcoHz: Double = 50_000_000.0 / 3.0 * 56.375
  val coreClockHz: Double = vcoHz / sysClockDivider
  /** DAC sample rate (~48 kHz, revision-dependent). */
  val outputRateHz: Double = vcoHz / (avClockDivider.toDouble * audioMclkFactor)

  /** CIC decimation ratio, targeting an intermediate rate of ~190.7 kHz (~4x the output
   *  rate). Both cores land on exactly the same intermediate rate: 8.39 MHz / 44 and
   *  16.78 MHz / 88 are both VCO / 4928. */
  val cicDecimation: Int = math.max(4, math.round(coreClockHz / 190_662.0).toInt)
  val intermediateRateHz: Double = coreClockHz / cicDecimation

  private val rawNum = avClockDivider * audioMclkFactor
  private val rawDen = sysClockDivider * cicDecimation
  private val divisor = BigInt(rawNum).gcd(BigInt(rawDen)).toInt

  /** Exact rational ratio intermediateRate / outputRate = phaseNum / phaseDen. */
  val phaseNum: Int = rawNum / divisor
  val phaseDen: Int = rawDen / divisor

  require(intermediateRateHz / outputRateHz > 3.0 && intermediateRateHz / outputRateHz < 6.0,
    "intermediate rate should be ~4x the output rate")
}

/**
 * Anti-aliased resampler from an emulator core's native-rate audio output to the DAC
 * sample rate.
 *
 * Signal chain:
 *
 *  1. '''3rd-order CIC decimator''' (core clock domain): samples the APU output every core
 *     clock cycle and decimates to an intermediate rate of ~4x the DAC rate (~190.7 kHz).
 *     Everything that can fold into a 20 kHz audio band through the later stages lies at
 *     >= 170 kHz here, where the CIC's sinc^3 response is >= 56 dB down.
 *
 *  2. '''Async FIFO''' (xpm_fifo_async, 16 deep): carries the intermediate-rate stream into
 *     the AV clock domain. Both clocks come from the same VCO, so occupancy is bounded.
 *
 *  3. '''75-tap Kaiser windowed-sinc FIR''' (AV clock domain, serial symmetric MAC, one DSP
 *     slice per channel): passband to 18 kHz, stopband from 28 kHz at ~-63 dB. Removes the
 *     supersonic content (24 kHz .. 95 kHz) that the fractional stage would fold into the
 *     audible band, and normalizes the CIC's non-power-of-two gain so the chain has unity
 *     gain overall. CIC passband droop is < 0.5 dB and is not compensated.
 *
 *  4. '''Catmull-Rom fractional resampler''' ([[RateInterpolator]]): produces one sample per
 *     transmitter sampleEnable at the exact rational phase increment; interpolation images
 *     are suppressed by >= ~55 dB at 4x oversampling.
 *
 *  5. '''DC blocker''' ([[DcBlocker]]): one-pole highpass (~7.5 Hz), the digital
 *     equivalent of the hardware's AC coupling.
 *
 * End-to-end latency is ~230 us, dominated by the FIR group delay (37 taps at 190.7 kHz).
 */

class AudioResampler(params: AudioResamplerParams) extends Module {
  private val inputBits = 16
  private val intermediateBits = 18
  private val coeffBits = 18
  private val firTaps = 75

  // Chain DC gain: ~1 dB below unity, leaving headroom for FIR ringing and cubic
  // interpolation overshoot ahead of the output saturation.
  private val chainDcGain = 0.89
  // The FIR normalizes the CIC's non-power-of-two gain to the chain target.
  private val firDcGain =
    chainDcGain / CicDecimator.gain(inputBits, order = 3, params.cicDecimation, intermediateBits)
  val firCoeffs: Seq[BigInt] = FirDesign.kaiserLowpass(
    numTaps = firTaps,
    cutoffNorm = 23_000.0 / params.intermediateRateHz,
    beta = 6.0,
    dcGain = firDcGain,
    coeffBits = coeffBits,
  )

  val io = IO(new Bundle {
    /** Emulator core clock domain */
    val coreClock = Input(Clock())
    val coreReset = Input(Bool())
    val coreLeft = Input(SInt(inputBits.W))
    val coreRight = Input(SInt(inputBits.W))

    /** AV clock domain (implicit module clock) */
    val sampleEnable = Input(Bool())
    /** Discards all buffered audio and restarts the chain (core reset, display switch). */
    val flush = Input(Bool())
    val left = Output(SInt(inputBits.W))
    val right = Output(SInt(inputBits.W))
  })

  // The flush covers both domains: the CIC front end and FIFO restart from the core
  // side, the filter chain from the AV side.
  val coreFlush = withClockAndReset(io.coreClock, io.coreReset) {
    XpmCdcSingle(clock, io.flush)
  }

  val fifo = Module(new XpmFifoAsync(UInt((2 * intermediateBits).W), 16))
  fifo.io.writeClock := io.coreClock
  fifo.io.readClock := clock
  fifo.io.reset := io.coreReset || coreFlush

  // Core clock domain: CIC decimation down to the intermediate rate.
  withClockAndReset(io.coreClock, io.coreReset || coreFlush) {
    val cicLeft = Module(new CicDecimator(inputBits, params.cicDecimation, intermediateBits))
    val cicRight = Module(new CicDecimator(inputBits, params.cicDecimation, intermediateBits))
    cicLeft.io.input := io.coreLeft
    cicRight.io.input := io.coreRight
    fifo.io.dataIn := Cat(cicLeft.io.output.bits.asUInt, cicRight.io.output.bits.asUInt)
    fifo.io.writeEnable := cicLeft.io.output.valid && !fifo.io.full && !fifo.io.writeResetBusy &&
      !coreFlush
  }

  withReset(reset.asBool || io.flush) {
    // AV clock domain: FIR filtering at the intermediate rate.
    val firLeft = Module(new FirFilter(intermediateBits, coeffBits, firCoeffs))
    val firRight = Module(new FirFilter(intermediateBits, coeffBits, firCoeffs))
    val firReady = firLeft.io.ready && firRight.io.ready
    val consume = !fifo.io.empty && !fifo.io.readResetBusy && firReady && !io.flush
    fifo.io.readEnable := consume
    firLeft.io.in.valid := consume
    firLeft.io.in.bits := fifo.io.dataOut(2 * intermediateBits - 1, intermediateBits).asSInt
    firRight.io.in.valid := consume
    firRight.io.in.bits := fifo.io.dataOut(intermediateBits - 1, 0).asSInt

    // Fractional resampling to the DAC rate.
    val interpolator = Module(new RateInterpolator(intermediateBits, params.phaseNum, params.phaseDen))
    interpolator.io.inValid := firLeft.io.out.valid
    interpolator.io.inLeft := firLeft.io.out.bits
    interpolator.io.inRight := firRight.io.out.bits
    interpolator.io.outEnable := io.sampleEnable

    def dcBlock(x: SInt, tick: Bool): SInt = {
      val blocker = Module(new DcBlocker(interpolator.outputBits))
      blocker.io.tick := tick
      blocker.io.in := x
      blocker.io.out
    }
    io.left := FirFilter.saturate(dcBlock(interpolator.io.outLeft, interpolator.io.outValid), inputBits)
    io.right := FirFilter.saturate(dcBlock(interpolator.io.outRight, interpolator.io.outValid), inputBits)
  }
}
