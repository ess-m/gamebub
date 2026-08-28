package lib.audio

import chisel3._

/**
 * One-pole DC blocking highpass: y[n] = x[n] - x[n-1] + (1 - 2^-poleShift) * y[n-1].
 *
 * The accumulator carries poleShift fractional bits, so the leak term is nonzero for any
 * residual above one fractional LSB and offsets of either sign decay below one output LSB.
 * Cutoff is fs * 2^-poleShift / (2 pi), ~7.5 Hz at 48 kHz with the default pole.
 *
 * The output carries one extra bit: a full-scale input step passes through transiently at
 * full amplitude on top of an existing swing.
 */

class DcBlocker(dataBits: Int, poleShift: Int = 10) extends Module {
  val outputBits = dataBits + 1
  private val accBits = outputBits + poleShift + 1

  val io = IO(new Bundle {
    /** Advances the filter by one sample period. */
    val tick = Input(Bool())
    val in = Input(SInt(dataBits.W))
    val out = Output(SInt(outputBits.W))
  })

  val xPrev = RegInit(0.S(dataBits.W))
  // Accumulator in Q(outputBits).(poleShift) fixed point.
  val y = RegInit(0.S(accBits.W))
  when (io.tick) {
    xPrev := io.in
    val next = ((io.in -& xPrev) << poleShift).asSInt +& (y - (y >> poleShift))
    y := next(accBits - 1, 0).asSInt
  }
  io.out := (y >> poleShift)(outputBits - 1, 0).asSInt
}
