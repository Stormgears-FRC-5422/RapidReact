package frc.utils.filters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * Based on edu.wpi.first.math.filter.SlewRateLimiter.
 * A class that limits the rate of change of an input value, but allows for limits to Jerk as
 * well as acceleration.
 * Like SlewRateLimiter, this is most useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 *
 * Though we may end up with a s-limited trapezoid profile as well...
 */

public class sCurveRateLimiter {
    private final double m_deltaRateLimit;
    private final double m_rateLimit;
    private double m_prevVal;
    private double m_prevTime;
    private double m_prevRateLimit;

    /**
     * Creates a new sCurveRateLimiter with the given rate limit and initial value.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     * @param deltaRateLimit The rate-of-change of rateLimit, in units per second per second.
     * @param initialValue The initial value of the input.
     */

    public sCurveRateLimiter(double rateLimit, double deltaRateLimit, double initialValue) {
        m_rateLimit = rateLimit;
        m_deltaRateLimit = deltaRateLimit;
        m_prevVal = initialValue;
        m_prevRateLimit = 0;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
     * Creates a new sCurveRateLimiter with the given rate limit and an initial value of zero.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public sCurveRateLimiter(double rateLimit, double deltaRateLimit) {
        this(rateLimit, deltaRateLimit,0.0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;

        // Basically the same calculation as below, but the input is the target rate limit (acceleration).
        m_prevRateLimit +=
                MathUtil.clamp(m_rateLimit - m_prevRateLimit, -m_deltaRateLimit * elapsedTime, m_deltaRateLimit * elapsedTime);

        // Don't want acceleration to grow without bound - its rails should be the actual rateLimit passed in
        m_prevRateLimit =
                MathUtil.clamp(m_prevRateLimit, -m_rateLimit, m_rateLimit);

        // the original code from the SlewRateLimiter, with the rate calculated above rather than the slew rate.
        m_prevVal +=
                MathUtil.clamp(input - m_prevVal, -m_prevRateLimit * elapsedTime, m_prevRateLimit * elapsedTime);

        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

}


