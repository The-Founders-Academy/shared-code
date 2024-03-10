package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.util.Timing;

public class SlewRateLimiter {
    private double m_positiveRateLimit, m_negativeRateLimit;
    private double m_lastValue;
    private double m_lastValueTimestamp = 0;
    private Timing.Timer m_elapsedTime;
    /**
     * Creates a new SlewRateLimiter with the given positive limits and initial value.
     * @param positiveRateLimit - The rate-of-change in the positive direction, in units per second.
     *                          This is expected to be positive.
     * @param negativeRateLimit - The rate-of-change in the negative direction, in units per second.
     *                          This is expected to be negative.
     * @param timer - A timer object used for timestamping values.
     */
    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue, Timing.Timer timer) {
        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
        m_lastValue = initialValue;
        m_elapsedTime = timer;
    }

    /**
     * Clamps the slew rate between the negative and positive limits.
     * @param input The current value of the controlled value.
     * @return The slewed value.
     */
    public double calculate(double input) {
        double delta = input - m_lastValue;
        double toAdd = MathUtil.clamp(delta,m_negativeRateLimit,m_positiveRateLimit);
        m_lastValue += toAdd;
        return m_lastValue;
    }

    public double getLastValue() {
        return m_lastValue;
    }
}
