package lib.audio

import chisel3._
import lib.util.EphemeralSimulator._
import org.scalatest.funsuite.AnyFunSuite

class CicDecimatorSpec extends AnyFunSuite {
  test("settles to the exact expected DC gain") {
    val decimation = 88
    val inputBits = 16
    val outputBits = 18
    simulate(new CicDecimator(inputBits, decimation, outputBits)) { dut =>
      dut.reset.poke(true)
      dut.clock.step()
      dut.reset.poke(false)

      val input = 12345
      dut.io.input.poke(input.S)
      // Let the integrator/comb cascade fully settle (order * decimation plus margin).
      dut.clock.step(decimation * 8)
      // Expected: input * decimation^3, truncated to the top outputBits.
      val shift = inputBits + CicDecimator.growth(3, decimation) - outputBits
      val expected = (BigInt(input) * BigInt(decimation).pow(3)) >> shift
      var checked = 0
      for (_ <- 0 until decimation * 3) {
        if (dut.io.output.valid.peek().litToBoolean) {
          assert(dut.io.output.bits.peek().litValue == expected)
          checked += 1
        }
        dut.clock.step()
      }
      assert(checked >= 2, "expected at least two valid output samples")
    }
  }
}

class FirFilterSpec extends AnyFunSuite {
  private val dataBits = 18
  private val coeffBits = 18
  // Same design point as the real instantiation (fs = 190.66 kHz).
  private val coeffs = FirDesign.kaiserLowpass(
    numTaps = 75,
    cutoffNorm = 23_000.0 / 190_662.0,
    beta = 6.0,
    dcGain = 1.0 / CicDecimator.gain(16, 3, 88, 18),
    coeffBits = coeffBits,
  )

  test("produces the exact expected DC response") {
    simulate(new FirFilter(dataBits, coeffBits, coeffs)) { dut =>
      dut.reset.poke(true)
      dut.clock.step()
      dut.reset.poke(false)

      val input = 30000
      val expected = (BigInt(input) * coeffs.sum) >> (coeffBits - 1)
      var lastOut: Option[BigInt] = None
      for (_ <- 0 until 100) {
        assert(dut.io.ready.peek().litToBoolean)
        dut.io.in.valid.poke(true)
        dut.io.in.bits.poke(input.S)
        dut.clock.step()
        dut.io.in.valid.poke(false)
        // Serial MAC: half (38) cycles plus output stage.
        for (_ <- 0 until 45) {
          if (dut.io.out.valid.peek().litToBoolean) {
            lastOut = Some(dut.io.out.bits.peek().litValue)
          }
          dut.clock.step()
        }
      }
      // After 75+ samples the delay line is fully primed with the constant.
      assert(lastOut.contains(expected), s"got $lastOut, expected $expected")
    }
  }

  test("designed coefficients are sane") {
    // DC gain must compensate the CIC gain to unity within quantization error.
    val dcGain = coeffs.sum.toDouble / (1L << (coeffBits - 1))
    val cicGain = CicDecimator.gain(16, 3, 88, 18)
    assert(math.abs(dcGain * cicGain - 1.0) < 1e-3)
  }

  test("quantized filter meets its passband/stopband spec") {
    val fs = 190_662.0
    def magnitudeDb(f: Double): Double = {
      val w = 2.0 * math.Pi * f / fs
      val re = coeffs.zipWithIndex.map { case (c, i) => c.toDouble * math.cos(w * i) }.sum
      val im = coeffs.zipWithIndex.map { case (c, i) => c.toDouble * math.sin(w * i) }.sum
      20.0 * math.log10(math.hypot(re, im) / coeffs.sum.toDouble)
    }
    // Passband: flat to well under the CIC's own ~0.4 dB droop.
    for (f <- 100 to 18000 by 500) {
      assert(math.abs(magnitudeDb(f)) < 0.05, s"passband ripple at $f Hz")
    }
    // Stopband: everything that could fold into the audible band in the fractional
    // stage (28 kHz up to the intermediate Nyquist) must be down by >= 60 dB.
    for (f <- 28000 to 95000 by 250) {
      assert(magnitudeDb(f) < -60.0, s"insufficient stopband rejection at $f Hz")
    }
  }
}

class RateInterpolatorSpec extends AnyFunSuite {
  test("tracks a ramp (cubic interpolation of a line is the line)") {
    // GBA rev C ratio: 190.66 kHz / 48.29 kHz = 304/77.
    simulate(new RateInterpolator(18, 304, 77)) { dut =>
      dut.reset.poke(true)
      dut.clock.step()
      dut.reset.poke(false)

      // Ramp with slope 100 per input sample.
      var pushed = 0
      def push(): Unit = {
        dut.io.inValid.poke(true)
        dut.io.inLeft.poke((pushed * 100).S)
        dut.io.inRight.poke((-pushed * 100).S)
        dut.clock.step()
        dut.io.inValid.poke(false)
        pushed += 1
      }
      for (_ <- 0 until 8) push()

      // The interpolator's initial window is s0..s3 = samples 0..3, interpolating
      // between s1 and s2. Each output advances the phase by 304/77 input samples.
      var phase = 1.0 // s1 position of the initial window
      for (n <- 0 until 12) {
        phase += 304.0 / 77.0
        // Keep the ring buffer topped up like the locked-rate FIR would.
        for (_ <- 0 until 4) push()
        dut.io.outEnable.poke(true)
        dut.clock.step()
        dut.io.outEnable.poke(false)
        var seen = false
        for (_ <- 0 until 20) {
          if (dut.io.outValid.peek().litToBoolean) seen = true
          dut.clock.step()
        }
        assert(seen, s"no output produced for sample $n")
        val expected = phase * 100.0
        val gotLeft = dut.io.outLeft.peek().litValue.toDouble
        val gotRight = dut.io.outRight.peek().litValue.toDouble
        assert(math.abs(gotLeft - expected) < 8.0, s"sample $n: left $gotLeft, expected ~$expected")
        assert(math.abs(gotRight + expected) < 8.0, s"sample $n: right $gotRight, expected ~${-expected}")
      }
    }
  }
}

class AudioResamplerElaborationSpec extends AnyFunSuite {
  test("elaborates for every core/revision combination") {
    for {
      sysDivider <- Seq(56, 112) // GBA/boot, Game Boy
      (avDivider, mclkFactor) <- Seq((76, 256), (36, 544), (32, 608)) // revisions A, B, C
    } {
      val params = AudioResamplerParams(sysDivider, avDivider, mclkFactor)
      _root_.circt.stage.ChiselStage.emitCHIRRTL(new AudioResampler(params))
    }
  }

  test("phase ratio is exact and small") {
    val params = AudioResamplerParams(56, 32, 608)
    assert(params.cicDecimation == 88)
    assert(params.phaseNum == 304 && params.phaseDen == 77)
    val gb = AudioResamplerParams(112, 32, 608)
    assert(gb.cicDecimation == 44)
    assert(gb.phaseNum == 304 && gb.phaseDen == 77)
  }
}
