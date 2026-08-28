package lib.audio

import chisel3._
import chisel3.util._

/**
 * Serial symmetric FIR filter: one multiply-accumulate per clock cycle, exploiting
 * linear-phase coefficient symmetry with a pre-adder (maps to a single DSP slice).
 *
 * A new input sample may be presented (valid && ready) at most once every
 * (numTaps / 2 + 3) cycles; one output sample is produced per input sample.
 */

class FirFilter(
  dataBits: Int,
  coeffBits: Int,
  coeffs: Seq[BigInt],
) extends Module {
  val numTaps = coeffs.length
  require(numTaps % 2 == 1, "symmetric odd-length FIR only")
  require(
    coeffs.zipWithIndex.forall { case (c, i) => c == coeffs(numTaps - 1 - i) },
    "coefficients must be symmetric",
  )
  // Number of unique coefficients (first half plus center tap).
  val half = numTaps / 2 + 1
  private val center = numTaps / 2
  private val idxBits = log2Ceil(numTaps)
  private val accBits = dataBits + 1 + coeffBits + log2Ceil(half)

  val io = IO(new Bundle {
    val in = Flipped(Valid(SInt(dataBits.W)))
    /** Whether a new input sample can be accepted this cycle. */
    val ready = Output(Bool())
    val out = Valid(SInt(dataBits.W))
  })

  val samples = Mem(numTaps, SInt(dataBits.W))
  val coeffRom = VecInit(coeffs.take(half).map(_.S(coeffBits.W)))

  // Index of the newest sample; incremented after each output is computed.
  val writeIndex = RegInit(0.U(idxBits.W))
  val busy = RegInit(false.B)
  val finishing = RegInit(false.B)
  val k = RegInit(0.U(log2Ceil(half).W))
  val acc = RegInit(0.S(accBits.W))
  io.ready := !busy && !finishing

  // (writeIndex - offset) mod numTaps, for offset < numTaps.
  private def tapIndex(offset: UInt): UInt = {
    val diff = writeIndex +& numTaps.U - offset
    Mux(diff >= numTaps.U, diff - numTaps.U, diff)(idxBits - 1, 0)
  }

  when (io.in.valid && io.ready) {
    samples.write(writeIndex, io.in.bits)
    busy := true.B
    k := 0.U
    acc := 0.S
  }

  finishing := false.B
  when (busy) {
    // Tap k pairs with its mirror tap (numTaps - 1 - k); the center tap stands alone.
    val x1 = samples.read(tapIndex(k))
    val x2 = samples.read(tapIndex((numTaps - 1).U - k))
    val pair = Mux(k === center.U, x1, x1 +& x2)
    acc := acc + (pair * coeffRom(k))
    when (k === center.U) {
      busy := false.B
      finishing := true.B
      writeIndex := Mux(writeIndex === (numTaps - 1).U, 0.U, writeIndex + 1.U)
    } .otherwise {
      k := k + 1.U
    }
  }

  val outReg = RegInit(0.S(dataBits.W))
  val outValid = RegInit(false.B)
  outValid := false.B
  when (finishing) {
    outReg := Saturate(acc >> (coeffBits - 1), dataBits)
    outValid := true.B
  }
  io.out.bits := outReg
  io.out.valid := outValid
}
