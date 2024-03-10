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
    protected abstract void updateRobotPose();
    protected abstract void updateDashboard();

    public BaseMecanumDrive(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, @NonNull Pose2d initialPose) {
        m_mecanumConfigs = mecanumConfigs;
        m_frontLeft = new MotorEx(hardwareMap, m_mecanumConfigs.frontLeftName(), Motor.GoBILDA.RPM_312);
        m_frontRight = new MotorEx(hardwareMap, m_mecanumConfigs.frontRightName(), Motor.GoBILDA.RPM_312);
        m_backLeft = new MotorEx(hardwareMap, m_mecanumConfigs.backLeftName(), Motor.GoBILDA.RPM_312);
        m_backRight = new MotorEx(hardwareMap, m_mecanumConfigs.backRightName(), Motor.GoBILDA.RPM_312);

        m_kinematics = new MecanumDriveKinematics(m_mecanumConfigs.frontLeftPosition(), m_mecanumConfigs.frontRightPosition(),
                m_mecanumConfigs.backLeftPosition(), m_mecanumConfigs.backRightPosition());

        m_odometry = new MecanumDriveOdometry(m_kinematics, initialPose.getRotation());



    }
    public void move(ChassisSpeeds speeds) {
        speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -1, 1) * m_mecanumConfigs.maxRobotSpeedMps();
        speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -1, 1) * m_mecanumConfigs.maxRobotSpeedMps();
        speeds.omegaRadiansPerSecond = MathUtil.clamp(speeds.omegaRadiansPerSecond, -1, 1) * m_mecanumConfigs.maxRobotRotationRps();

        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        m_frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond);
        m_frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond);
        m_backLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond);
        m_backRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond);
    }

    public void moveRobotRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        ChassisSpeeds speeds = new ChassisSpeeds(xPercentVelocity, yPercentVelocity, omegaPercentVelocity);
        move(speeds);
    }

    public void moveFieldRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xPercentVelocity, yPercentVelocity, omegaPercentVelocity, getHeading());
        move(speeds);
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(pose, getHeading());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        updateDashboard();
        updateRobotPose();
    }
}
