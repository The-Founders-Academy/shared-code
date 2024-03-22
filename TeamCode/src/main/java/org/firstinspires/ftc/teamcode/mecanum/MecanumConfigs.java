package org.firstinspires.ftc.teamcode.mecanum;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
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
    private Motor.RunMode m_runMode = Motor.RunMode.VelocityControl;
    private String m_frontLeftName = "fL";
    private String m_frontRightName = "fR";
    private String m_backLeftName = "bL";
    private String m_backRightName = "bR";

    public MecanumConfigs() {
    }

    public MecanumConfigs TranslationPIDValues(PIDCoefficients translationPIDValues) {
        m_translationPIDValues = translationPIDValues;
        return this;
    }

    public MecanumConfigs RotationPIDValues(PIDCoefficients rotationPIDValues) {
        m_rotationPIDValues = rotationPIDValues;
        return this;
    }

    public MecanumConfigs DriveToTargetToleranceMeters(double driveToTargetToleranceMeters) {
        m_driveToTargetToleranceMeters = driveToTargetToleranceMeters;
        return this;
    }

    public MecanumConfigs DriveToTargetToleranceRadians(double driveToTargetToleranceRadians) {
        m_driveToTargetToleranceRadians = driveToTargetToleranceRadians;
        return this;
    }

    public MecanumConfigs maxRobotSpeedMps(double maxRobotSpeedMps) {
        m_maxRobotSpeedMps = maxRobotSpeedMps;
        return this;
    }

    public MecanumConfigs maxRobotRotationRps(double maxRobotRotationRps) {
        m_maxRobotRotationRps = maxRobotRotationRps;
        return this;
    }

    public MecanumConfigs metersPerTick(double mpt) {
        m_metersPerTick = mpt;
        return this;
    }

    public MecanumConfigs wheelPIDValues(PIDCoefficients pids) {
        m_wheelPIDValues = pids;
        return this;
    }

    public MecanumConfigs runMode(Motor.RunMode mode) {
        m_runMode = mode;
        return this;
    }

    public PIDCoefficients getTranslationPIDValues() {
        return m_translationPIDValues;
    }

    public PIDCoefficients getRotationPIDValues() {
        return m_rotationPIDValues;
    }

    public double getDriveToTargetToleranceMeters() {
        return m_driveToTargetToleranceMeters;
    }

    public double getDriveToTargetToleranceRadians() {
        return m_driveToTargetToleranceRadians;
    }

    public double getMaxRobotSpeedMps() {
        return m_maxRobotSpeedMps;
    }

    public double getMaxRobotRotationRps() {
        return m_maxRobotRotationRps;
    }

    public String getFrontLeftName() {
        return m_frontLeftName;
    }

    public String getFrontRightName() {
        return m_frontRightName;
    }

    public String getBackLeftName() {
        return m_backLeftName;
    }

    public String getBackRightName() {
        return m_backRightName;
    }

    public Translation2d getFrontLeftPosition() {
        return m_frontLeftPositionMeters;
    }

    public Translation2d getFrontRightPosition() {
        return m_frontRightPositionMeters;
    }

    public Translation2d getBackLeftPosition() {
        return m_backLeftPositionMeters;
    }

    public Translation2d getBackRightPosition() {
        return m_backRightPositionMeters;
    }

    public double getMetersPerTick() {
        return m_metersPerTick;
    }

    public PIDCoefficients getWheelPIDValues() {
        return m_wheelPIDValues;
    }

    public Motor.RunMode getRunMode() {
        return m_runMode;
    }
}
