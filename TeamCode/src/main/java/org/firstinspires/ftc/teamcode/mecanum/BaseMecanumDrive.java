package org.firstinspires.ftc.teamcode.mecanum;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.Optional;

public abstract class BaseMecanumDrive extends SubsystemBase {
    protected MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    protected MecanumDriveKinematics m_kinematics;
    protected MecanumDriveOdometry m_odometry;
    protected MecanumConfigs m_mecanumConfigs;

    public abstract Rotation2d getHeading();

    public BaseMecanumDrive(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose) {
        m_mecanumConfigs = mecanumConfigs;
        m_frontLeft = new MotorEx(hardwareMap, m_mecanumConfigs.frontLeftName(), Motor.GoBILDA.RPM_312);
        m_frontRight = new MotorEx(hardwareMap, m_mecanumConfigs.frontRightName(), Motor.GoBILDA.RPM_312);
        m_backLeft = new MotorEx(hardwareMap, m_mecanumConfigs.backLeftName(), Motor.GoBILDA.RPM_312);
        m_backRight = new MotorEx(hardwareMap, m_mecanumConfigs.backRightName(), Motor.GoBILDA.RPM_312);

        m_frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        m_frontRight.setRunMode(Motor.RunMode.VelocityControl);
        m_backLeft.setRunMode(Motor.RunMode.VelocityControl);
        m_backRight.setRunMode(Motor.RunMode.VelocityControl);

        m_kinematics = new MecanumDriveKinematics(m_mecanumConfigs.frontLeftPosition(), m_mecanumConfigs.frontRightPosition(),
                m_mecanumConfigs.backLeftPosition(), m_mecanumConfigs.backRightPosition());

        m_odometry = new MecanumDriveOdometry(m_kinematics, initialPose.getRotation());
    }

    protected void move(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        m_frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond);
        m_frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond);
        m_backLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond);
        m_backRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond);
    }

    public void moveRobotRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        double xVelMps = xPercentVelocity * m_mecanumConfigs.maxRobotSpeedMps();
        double yVelMps = yPercentVelocity * m_mecanumConfigs.maxRobotSpeedMps();
        double omegaVelRps = omegaPercentVelocity * m_mecanumConfigs.maxRobotRotationRps();
        ChassisSpeeds speeds = new ChassisSpeeds(xVelMps, yVelMps, omegaVelRps);
        move(speeds);
    }

    public void moveFieldRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        double xVelMps = xPercentVelocity * m_mecanumConfigs.maxRobotSpeedMps();
        double yVelMps = yPercentVelocity * m_mecanumConfigs.maxRobotSpeedMps();
        double omegaVelRps = omegaPercentVelocity * m_mecanumConfigs.maxRobotRotationRps();
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelMps, yVelMps, omegaVelRps, getHeading());
        move(speeds);
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(pose, getHeading());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

}
