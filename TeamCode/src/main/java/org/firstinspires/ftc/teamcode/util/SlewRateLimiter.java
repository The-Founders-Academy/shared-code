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
     * @return The slewed rate.
     */
    public double calculate(double input) {
        if(m_lastValue < input) {
            m_lastValue += 0.075;
            return m_lastValue;
        } else if(m_lastValue > input) {
            m_lastValue -= 0.075;
            return m_lastValue;
        } else {
            return input;
        }
    }

    public double getLastValue() {
        return m_lastValue;
    }
}
