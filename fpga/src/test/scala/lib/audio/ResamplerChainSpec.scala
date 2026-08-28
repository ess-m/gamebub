package lib.audio

import chisel3._
import chisel3.util._
import lib.util.EphemeralSimulator._
import org.scalatest.funsuite.AnyFunSuite

/**
 * Full-chain harness at GBA rev4 rates, single clock domain (the async FIFO is
 * bypassed; rates and logic are otherwise identical to AudioResampler).
 *
 * A square-wave generator drives the CIC at the core rate; an exact rational
 * accumulator fires outEnable at the DAC rate (7/2432 of core cycles).
 */

class ResamplerChainHarness(squareHalfPeriod: Int, coeffs: Seq[BigInt]) extends Module {
  val io = IO(new Bundle {
    val out = Output(SInt(19.W))
    val outSeq = Output(UInt(32.W))
  })

  // Square wave generator: +/-16384 at the core clock rate.
  val phase = RegInit(0.U(16.W))
  val level = RegInit(false.B)
  when (phase === (squareHalfPeriod - 1).U) {
    phase := 0.U
    level := !level
  } .otherwise {
    phase := phase + 1.U
  }
  val square = Mux(level, 16384.S(16.W), (-16384).S(16.W))

  val cic = Module(new CicDecimator(16, 88, 18))
  cic.io.input := square

  val fir = Module(new FirFilter(18, 18, coeffs))
  fir.io.in.valid := cic.io.output.valid
  fir.io.in.bits := cic.io.output.bits
  assert(!(cic.io.output.valid && !fir.io.ready), "FIR not ready for CIC sample")

  val interp = Module(new RateInterpolator(18, 304, 77))
  interp.io.inValid := fir.io.out.valid
  interp.io.inLeft := fir.io.out.bits
  interp.io.inRight := fir.io.out.bits

  // outEnable at exactly (7/2432) * coreClock = 48293.08 Hz.
  val acc = RegInit(0.U(13.W))
  val next = acc +& 7.U
  val fire = next >= 2432.U
  acc := Mux(fire, next - 2432.U, next)
  interp.io.outEnable := fire

  val outReg = RegInit(0.S(19.W))
  val outSeq = RegInit(0.U(32.W))
  when (interp.io.outValid) {
    outReg := interp.io.outLeft
    outSeq := outSeq + 1.U
  }
  io.out := outReg
  io.outSeq := outSeq
}

class ResamplerChainSpec extends AnyFunSuite {
  private val coreHz = 50_000_000.0 / 3.0 * 56.375 / 56.0
  private val outHz = coreHz * 7.0 / 2432.0

  private val chainDcGain = 0.89
  private val coeffs = FirDesign.kaiserLowpass(
    numTaps = 75,
    cutoffNorm = 23_000.0 / (coreHz / 88.0),
    beta = 6.0,
    dcGain = chainDcGain / CicDecimator.gain(16, 3, 88, 18),
    coeffBits = 18,
  )

  /** Hann-windowed Goertzel amplitude at frequency f (Hz) over samples at rate fs. */
  private def goertzel(samples: Seq[Double], f: Double, fs: Double): Double = {
    val n = samples.length
    val windowed = samples.zipWithIndex.map { case (s, i) =>
      s * 0.5 * (1.0 - math.cos(2.0 * math.Pi * i / (n - 1)))
    }
    val w = 2.0 * math.Pi * f / fs
    var re = 0.0
    var im = 0.0
    for ((s, i) <- windowed.zipWithIndex) {
      re += s * math.cos(w * i)
      im += s * math.sin(w * i)
    }
    // Hann coherent gain = 0.5
    2.0 * math.hypot(re, im) / (n * 0.5)
  }

  test("full chain kills aliased square-wave harmonics that point-sampling folds into the audio band") {
    // Square at ~5 kHz: fundamental passes; its 9th harmonic (~45 kHz) would fold
    // to ~3.3 kHz at the DAC rate under the old point-sampling path.
    val halfPeriod = 1678
    val f0 = coreHz / (2.0 * halfPeriod)
    val h9 = 9.0 * f0
    val f9alias = math.abs(h9 - outHz)

    val numSamples = 4096
    val captured = new scala.collection.mutable.ArrayBuffer[Double](numSamples)

    simulate(new ResamplerChainHarness(halfPeriod, coeffs)) { dut =>
      dut.reset.poke(true)
      dut.clock.step()
      dut.reset.poke(false)

      // Settle: CIC/FIR group delay plus interpolator prefill.
      dut.clock.step(88 * 300)

      var lastSeq = dut.io.outSeq.peek().litValue
      while (captured.length < numSamples) {
        dut.clock.step(100)
        val seq = dut.io.outSeq.peek().litValue
        if (seq != lastSeq) {
          // 100-cycle granularity vs 347-cycle sample period: at most one new sample per poll.
          assert(seq - lastSeq == 1, s"missed samples: $lastSeq -> $seq")
          captured += dut.io.out.peek().litValue.toDouble
          lastSeq = seq
        }
      }
    }

    // Old path for comparison: point-sample the same ideal square at the DAC instants.
    val oldPath = (0 until numSamples).map { i =>
      val t = i / outHz
      val x = t * f0
      if ((x - math.floor(x)) < 0.5) 16384.0 else -16384.0
    }

    val newFund = goertzel(captured.toSeq, f0, outHz)
    val newAlias = goertzel(captured.toSeq, f9alias, outHz)
    val oldFund = goertzel(oldPath, f0, outHz)
    val oldAlias = goertzel(oldPath, f9alias, outHz)

    def db(x: Double) = 20.0 * math.log10(x)
    println(f"square fundamental f0 = $f0%.1f Hz, 9th harmonic alias lands at $f9alias%.1f Hz")
    println(f"OLD path: fundamental ${db(oldFund)}%.1f dB, alias ${db(oldAlias)}%.1f dB (rel: ${db(oldAlias / oldFund)}%.1f dB)")
    println(f"NEW path: fundamental ${db(newFund)}%.1f dB, alias ${db(newAlias)}%.1f dB (rel: ${db(newAlias / newFund)}%.1f dB)")

    // Interpolator phase-slip check: an occupancy stall in the consume pattern
    // phase-modulates the signal at 4 * fOut / 77 (~2509 Hz) and its harmonics,
    // producing sidebands at that comb +/- f0.
    val slipRate = 4.0 * outHz / 77.0
    for (k <- 1 to 2) {
      val lo = goertzel(captured.toSeq, k * slipRate - f0, outHz)
      val hi = goertzel(captured.toSeq, k * slipRate + f0, outHz)
      println(f"slip comb k=$k (${k * slipRate}%.1f Hz): sidebands ${db(lo / newFund)}%.1f / ${db(hi / newFund)}%.1f dB rel")
      assert(db(lo / newFund) < -70.0 && db(hi / newFund) < -70.0,
        f"phase-slip sidebands present at k=$k: ${db(lo / newFund)}%.1f / ${db(hi / newFund)}%.1f dB")
    }

    // Fundamental must pass at the chain gain (square fundamental = 4/pi * amplitude).
    val expectedFund = 16384.0 * 4.0 / math.Pi * chainDcGain
    assert(math.abs(newFund - expectedFund) / expectedFund < 0.1,
      s"fundamental level wrong: $newFund vs $expectedFund")
    // The old path folds the 9th harmonic at roughly -19 dB rel; the new path must
    // suppress it by at least 35 dB more.
    assert(db(oldAlias / oldFund) > -25.0, "test invalid: old path should show a strong alias")
    assert(db(newAlias / newFund) < -55.0,
      f"insufficient alias suppression: ${db(newAlias / newFund)}%.1f dB")
  }
}
