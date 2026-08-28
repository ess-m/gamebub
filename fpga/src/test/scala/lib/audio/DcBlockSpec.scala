package lib.audio

import chisel3._
import lib.util.EphemeralSimulator._
import org.scalatest.funsuite.AnyFunSuite

class DcBlockSpec extends AnyFunSuite {
  private val dataBits = 19

  test("passband polarity is positive") {
    simulate(new DcBlocker(dataBits)) { dut =>
      dut.reset.poke(true); dut.clock.step(); dut.reset.poke(false)
      dut.io.tick.poke(true)
      // ~500 Hz sine at a 48 kHz tick rate
      var corr = 0.0
      for (n <- 0 until 2000) {
        val x = (20000 * math.sin(2 * math.Pi * 500 * n / 48000.0)).toInt
        dut.io.in.poke(x.S)
        dut.clock.step()
        if (n > 200) corr += x.toDouble * dut.io.out.peek().litValue.toDouble
      }
      assert(corr > 0, "DC blocker inverts")
    }
  }

  test("DC steps of either sign decay below one output LSB") {
    simulate(new DcBlocker(dataBits)) { dut =>
      dut.reset.poke(true); dut.clock.step(); dut.reset.poke(false)
      dut.io.tick.poke(true)
      for (step <- Seq(20000, -20000)) {
        dut.io.in.poke(step.S)
        // Pole time constant is 2^10 ticks; run well past settling.
        dut.clock.step(16 * 1024)
        val out = dut.io.out.peek().litValue
        assert(out.abs <= 1, s"DC step $step left residual $out")
      }
    }
  }
}
