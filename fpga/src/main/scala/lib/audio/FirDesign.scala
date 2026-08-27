package lib.audio

/**
 * Elaboration-time FIR filter design (plain Scala, no hardware).
 */

object FirDesign {
  // Zeroth-order modified Bessel function of the first kind (power series).
  private def besselI0(x: Double): Double = {
    var sum = 1.0
    var term = 1.0
    var k = 1
    while (term > sum * 1e-12) {
      val t = x / (2.0 * k)
      term *= t * t
      sum += term
      k += 1
    }
    sum
  }

  private def sinc(x: Double): Double =
    if (x == 0.0) 1.0 else math.sin(math.Pi * x) / (math.Pi * x)

  /**
   * Design a symmetric (linear-phase) Kaiser-windowed-sinc lowpass filter.
   *
   * @param numTaps    filter length; must be odd
   * @param cutoffNorm -6 dB cutoff frequency, normalized to the sample rate (fc / fs)
   * @param beta       Kaiser window shape parameter (6.0 gives roughly -63 dB stopband)
   * @param dcGain     desired gain at DC (sum of coefficients)
   * @param coeffBits  coefficients are quantized to signed fixed point Q1.(coeffBits-1)
   * @return quantized coefficients, i.e. round(h * 2^(coeffBits-1))
   */
  def kaiserLowpass(
    numTaps: Int,
    cutoffNorm: Double,
    beta: Double,
    dcGain: Double,
    coeffBits: Int,
  ): Seq[BigInt] = {
    require(numTaps % 2 == 1, "use an odd tap count for a symmetric filter with integer group delay")
    require(cutoffNorm > 0.0 && cutoffNorm < 0.5)
    val m = numTaps - 1
    val raw = (0 until numTaps).map { i =>
      val r = 2.0 * i / m - 1.0
      val window = besselI0(beta * math.sqrt(math.max(0.0, 1.0 - r * r))) / besselI0(beta)
      2.0 * cutoffNorm * sinc(2.0 * cutoffNorm * (i - m / 2)) * window
    }
    val scale = dcGain / raw.sum * (1L << (coeffBits - 1))
    val quantized = raw.map(c => BigInt(math.round(c * scale)))
    val maxCoeff = (BigInt(1) << (coeffBits - 1)) - 1
    require(quantized.forall(c => c.abs <= maxCoeff), "coefficient overflow; reduce dcGain or increase coeffBits")
    quantized
  }
}
