package lib.audio

import chisel3._
import lib.util.EphemeralSimulator._
import org.scalatest.funsuite.AnyFunSuite

/** Replicates the dcBlock + saturate output stage of AudioResampler exactly. */
class DcBlockHarness extends Module {
  val outputBits = 19
  val io = IO(new Bundle {
    val x = Input(SInt(outputBits.W))
    val tick = Input(Bool())
    val out = Output(SInt(16.W))
  })
  val xPrev = RegInit(0.S(outputBits.W))
  val y = RegInit(0.S((outputBits + 3).W))
  when (io.tick) {
    xPrev := io.x
    y := (io.x -& xPrev) +& (y - (y >> 10).asSInt)
  }
  io.out := FirFilter.saturate(y, 16)
}

class DcBlockSpec extends AnyFunSuite {
  test("passband polarity is positive") {
    simulate(new DcBlockHarness) { dut =>
      dut.reset.poke(true); dut.clock.step(); dut.reset.poke(false)
      dut.io.tick.poke(true)
      // ~500 Hz sine at 48 kHz tick rate, amplitude 20000
      var corr = 0.0
      for (n <- 0 until 2000) {
        val x = (20000 * math.sin(2 * math.Pi * 500 * n / 48000.0)).toInt
        dut.io.x.poke(x.S)
        dut.clock.step()
        if (n > 200) corr += x.toDouble * dut.io.out.peek().litValue.toDouble
      }
      println(s"input/output correlation sum: $corr")
      assert(corr > 0, "DC blocker/saturate stage inverts!")
    }
  }
}
