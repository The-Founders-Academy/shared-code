package org.firstinspires.ftc.teamcode.mecanum;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class MecanumConfigs {
    // Default values
    private PIDCoefficients m_wheelPIDValues = new PIDCoefficients(0.1, 0, 0);
    private PIDCoefficients m_translationPIDValues = new PIDCoefficients(0.1, 0, 0);
    private PIDCoefficients m_rotationPIDValues = new PIDCoefficients(0.1, 0, 0);
    private Translation2d m_frontLeftPositionMeters = new Translation2d(0.178, 0.168);
    private Translation2d m_frontRightPositionMeters = new Translation2d(0.178, -0.168);
    private Translation2d m_backLeftPositionMeters = new Translation2d(-0.178, 0.168);
    private Translation2d m_backRightPositionMeters = new Translation2d(-0.178, -0.168);
    private double m_driveToTargetToleranceMeters = 0.3;
    private double m_driveToTargetToleranceRadians = 0.1;
    private double m_maxRobotSpeedMps = 3;
    private double m_maxRobotRotationRps = 6.28;
    private double m_metersPerTick = 0.00056;
    private String m_frontLeftName = "fL";
    private String m_frontRightName = "fR";
    private String m_backLeftName = "bL";
    private String m_backRightName = "bR";

    public MecanumConfigs() {
    }

    public PIDCoefficients translationPIDValues() {
        return m_translationPIDValues;
    }

    public MecanumConfigs setTranslationPIDValues(PIDCoefficients translationPIDValues) {
        m_translationPIDValues = translationPIDValues;
        return this;
    }

    public PIDCoefficients rotationPIDValues() {
        return m_rotationPIDValues;
    }

    public MecanumConfigs setRotationPIDValues(PIDCoefficients rotationPIDValues) {
        m_rotationPIDValues = rotationPIDValues;
        return this;
    }

    public double driveToTargetToleranceMeters() {
        return m_driveToTargetToleranceMeters;
    }

    public MecanumConfigs setDriveToTargetToleranceMeters(double driveToTargetToleranceMeters) {
        m_driveToTargetToleranceMeters = driveToTargetToleranceMeters;
        return this;
    }

    public double driveToTargetToleranceRadians() {
        return m_driveToTargetToleranceRadians;
    }

    public MecanumConfigs setDriveToTargetToleranceRadians(double driveToTargetToleranceRadians) {
        m_driveToTargetToleranceRadians = driveToTargetToleranceRadians;
        return this;
    }

    public double maxRobotSpeedMps() {
        return m_maxRobotSpeedMps;
    }

    public MecanumConfigs setMaxRobotSpeedMps(double maxRobotSpeedMps) {
        m_maxRobotSpeedMps = maxRobotSpeedMps;
        return this;
    }

    public double maxRobotRotationRps() {
        return m_maxRobotRotationRps;
    }

    public MecanumConfigs setMaxRobotRotationRps(double maxRobotRotationRps) {
        m_maxRobotRotationRps = maxRobotRotationRps;
        return this;
    }

    public MecanumConfigs metersPerTick(double mpt) {
        m_metersPerTick = mpt;
        return this;
    }

    public String frontLeftName() {
        return m_frontLeftName;
    }

    public String frontRightName() {
        return m_frontRightName;
    }

    public String backLeftName() {
        return m_backLeftName;
    }

    public String backRightName() {
        return m_backRightName;
    }

    public Translation2d frontLeftPosition() {
        return m_frontLeftPositionMeters;
    }

    public Translation2d frontRightPosition() {
        return m_frontRightPositionMeters;
    }

    public Translation2d backLeftPosition() {
        return m_backLeftPositionMeters;
    }

    public Translation2d backRightPosition() {
        return m_backRightPositionMeters;
    }

    public double getMetersPertick() {
        return m_metersPerTick;
    }
}
