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

import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.Optional;

public abstract class BaseMecanumDrive extends SubsystemBase {
    protected MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    protected MecanumDriveKinematics m_kinematics;
    protected MecanumConfigs m_mecanumConfigs;

    public abstract Rotation2d getHeading();
    public abstract Pose2d getPose();
    public abstract void resetPose(Pose2d pose);

    public BaseMecanumDrive(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose) {
        m_mecanumConfigs = mecanumConfigs;
        m_frontLeft = new MotorEx(hardwareMap, m_mecanumConfigs.getFrontLeftName(), Motor.GoBILDA.RPM_312);
        m_frontRight = new MotorEx(hardwareMap, m_mecanumConfigs.getFrontRightName(), Motor.GoBILDA.RPM_312);
        m_backLeft = new MotorEx(hardwareMap, m_mecanumConfigs.getBackLeftName(), Motor.GoBILDA.RPM_312);
        m_backRight = new MotorEx(hardwareMap, m_mecanumConfigs.getBackRightName(), Motor.GoBILDA.RPM_312);

        m_frontLeft.setRunMode(m_mecanumConfigs.getRunMode());
        m_frontRight.setRunMode(m_mecanumConfigs.getRunMode());
        m_backLeft.setRunMode(m_mecanumConfigs.getRunMode());
        m_backRight.setRunMode(m_mecanumConfigs.getRunMode());

        m_kinematics = new MecanumDriveKinematics(m_mecanumConfigs.getFrontLeftPosition(), m_mecanumConfigs.getFrontRightPosition(),
                m_mecanumConfigs.getBackLeftPosition(), m_mecanumConfigs.getBackRightPosition());

    }

    protected void move(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        m_frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
        m_frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
        m_backLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
        m_backRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
    }

    public void moveRobotRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        double vXMps = xPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double vYMps = yPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double omegaRps = omegaPercentVelocity * m_mecanumConfigs.getMaxRobotRotationRps();
        ChassisSpeeds speeds = new ChassisSpeeds(vXMps, vYMps, omegaRps);
        move(speeds);
    }

    public void moveFieldRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        double vXMps = xPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double vYMps = yPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double omegaRps = omegaPercentVelocity * m_mecanumConfigs.getMaxRobotRotationRps();
        ChassisSpeeds speeds;
        if(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.BLUE) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vXMps, vYMps, omegaRps, getHeading().minus(Rotation2d.fromDegrees(180)));
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vXMps, vYMps, omegaRps, getHeading());
        }
        move(speeds);
    }
}
