package lib.audio

import chisel3._
import chisel3.util._

object CicDecimator {
  /** Internal register bit growth for a given order/ratio (log2 of the filter's DC gain, rounded up). */
  def growth(order: Int, decimation: Int): Int =
    (BigInt(decimation).pow(order) - 1).bitLength

  /** Overall DC gain from an `inputBits` input to the truncated `outputBits` output. */
  def gain(inputBits: Int, order: Int, decimation: Int, outputBits: Int): Double = {
    val shift = inputBits + growth(order, decimation) - outputBits
    BigInt(decimation).pow(order).toDouble / math.pow(2.0, shift)
  }
}

/**
 * CIC (cascaded integrator-comb) decimation filter: samples the input every clock cycle and
 * produces one output sample every `decimation` cycles.
 *
 * The frequency response is sinc(f/fOut)^order, with nulls on every multiple of the output
 * rate -- the frequencies that would alias to DC. Implemented with adders only; the
 * integrators deliberately wrap (standard CIC modular arithmetic).
 *
 * The output is the top `outputBits` of the last comb stage, giving a DC gain of
 * decimation^order / 2^(inputBits + growth - outputBits), slightly above unity. Downstream
 * filtering is expected to normalize this (see [[CicDecimator.gain]]).
 */

class CicDecimator(
  inputBits: Int,
  decimation: Int,
  outputBits: Int,
  order: Int = 3,
) extends Module {
  val width = inputBits + CicDecimator.growth(order, decimation)
  private val shift = width - outputBits
  require(shift >= 0)

  val io = IO(new Bundle {
    /** Input signal, sampled every clock cycle. */
    val input = Input(SInt(inputBits.W))
    /** Decimated output; valid pulses once every `decimation` cycles. */
    val output = Valid(SInt(outputBits.W))
  })

  // Integrator cascade, updated every cycle. Each stage registers its output, adding
  // one sample of delay per stage; the magnitude response is unaffected.
  val integrators = RegInit(VecInit(Seq.fill(order)(0.S(width.W))))
  integrators(0) := integrators(0) + io.input
  for (i <- 1 until order) {
    integrators(i) := integrators(i) + integrators(i - 1)
  }

  // Comb cascade, updated at the decimated rate.
  val counter = Counter(decimation)
  val strobe = counter.inc()
  val combDelays = RegInit(VecInit(Seq.fill(order)(0.S(width.W))))
  val outReg = RegInit(0.S(outputBits.W))
  val outValid = RegInit(false.B)
  outValid := false.B
  when (strobe) {
    var stage = integrators(order - 1)
    for (i <- 0 until order) {
      val next = stage - combDelays(i)
      combDelays(i) := stage
      stage = next
    }
    outReg := (stage >> shift)(outputBits - 1, 0).asSInt
    outValid := true.B
  }
  io.output.bits := outReg
  io.output.valid := outValid
}
