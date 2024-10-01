package org.firstinspires.ftc.teamcode.mecanum;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MathUtil;


public abstract class BaseMecanumDrive extends SubsystemBase {

    public enum Alliance {
        RED, BLUE
    }

    protected SimpleMotorFeedforward m_frontLeftFF, m_frontRightFF, m_backLeftFF, m_backRightFF;
    protected MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    protected MecanumDriveKinematics m_kinematics;
    protected MecanumConfigs m_mecanumConfigs;
    protected Alliance m_alliance;
    protected Pose2d m_robotPose;

    public abstract Rotation2d getHeading();
    public abstract Pose2d getPose();
    public abstract void resetPose(Pose2d pose);

    protected double m_kS = 0;
    protected double m_kV = 0;
    protected double m_kA = 0;

    /*
  This is how the PID loop will look in pseudocode.

  ... set PID setpoints up here

  setTargetPose(somePose);
  while not atTargetPose():
      moveWithPID();
      tunePIDs();
  resetPIDs();

   */
    protected PIDController m_translationXController = new PIDController(0, 0, 0);
    protected PIDController m_translationYController = new PIDController(0, 0, 0);;
    protected PIDController m_rotationController = new PIDController(0, 0, 0);;


    public BaseMecanumDrive(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose, Alliance alliance) {
        resetFeedForward(0 , 0);

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

        m_alliance = alliance;
    }

    protected void move(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        m_frontLeft.setVelocity(m_frontLeftFF.calculate(wheelSpeeds.frontLeftMetersPerSecond * m_mecanumConfigs.getTicksPerMeter()));
        m_frontRight.setVelocity(m_frontRightFF.calculate(wheelSpeeds.frontRightMetersPerSecond * m_mecanumConfigs.getTicksPerMeter()));
        m_backLeft.setVelocity(m_backLeftFF.calculate(wheelSpeeds.rearLeftMetersPerSecond * m_mecanumConfigs.getTicksPerMeter()));
        m_backRight.setVelocity(m_backRightFF.calculate(wheelSpeeds.rearRightMetersPerSecond * m_mecanumConfigs.getTicksPerMeter()));
    }

    /**
     * @param xPercentVelocity The forward velocity. Ranges from -1 to 1.
     * @param yPercentVelocity The leftward (from the driverstation) velocity. Ranges from -1 to 1.
     * @param omegaPercentVelocity The rotational velocity. Positive indicates cc rotation. Ranges from -1 to 1.
     */
    public void moveRobotRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        double vXMps = xPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double vYMps = yPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double omegaRps = omegaPercentVelocity * m_mecanumConfigs.getMaxRobotRotationRps();
        ChassisSpeeds speeds = new ChassisSpeeds(vXMps, vYMps, omegaRps);
        move(speeds);
    }

    /**
     * @param xPercentVelocity The forward velocity. Ranges from -1 to 1.
     * @param yPercentVelocity The leftward (from the driverstation) velocity. Ranges from -1 to 1.
     * @param omegaPercentVelocity The rotational velocity. Positive indicates cc rotation. Ranges from -1 to 1.
     */
    public void moveFieldRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        double vXMps = xPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();

        double vYMps = yPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double omegaRps = omegaPercentVelocity * m_mecanumConfigs.getMaxRobotRotationRps();
        ChassisSpeeds speeds;
        if(m_alliance == Alliance.BLUE) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vXMps, vYMps, omegaRps, getHeading().minus(Rotation2d.fromDegrees(180)));
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vXMps, vYMps, omegaRps, getHeading());
        }
        move(speeds);
    }

    public void moveFieldRelativeForPID() {
        double vX = MathUtil.clamp(m_translationXController.calculate(m_robotPose.getX()),
                -m_mecanumConfigs.getMaxRobotSpeedMps(),
                m_mecanumConfigs.getMaxRobotSpeedMps());
        double vY = MathUtil.clamp(m_translationYController.calculate(m_robotPose.getY()),
                -m_mecanumConfigs.getMaxRobotSpeedMps(),
                m_mecanumConfigs.getMaxRobotSpeedMps());

        // Do some angle wrapping to ensure the shortest path is taken to get to the rotation target
        double normalizedRotationRad = m_robotPose.getHeading();
        if(normalizedRotationRad < 0) {
            normalizedRotationRad = m_robotPose.getHeading() + 2 * Math.PI; // Normalize to [0, 2PI]
        }

        double vOmega = MathUtil.clamp(m_rotationController.calculate(normalizedRotationRad),
                -m_mecanumConfigs.getMaxRobotRotationRps(),
                m_mecanumConfigs.getMaxRobotRotationRps());

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vY, -vX, vOmega, getHeading()); // Transform the x and y coordinates to account for differences between global field coordinates and driver field coordinates
        move(speeds);
    }

    /**
     * Simultaneously updates all four motor feedforward kS and kV values.
     * @param kS The desired kS value.
     * @param kV The desired kV value.
     */
    protected void resetFeedForward(double kS, double kV) {
        m_frontLeftFF = new SimpleMotorFeedforward(kS, kV);
        m_frontRightFF = new SimpleMotorFeedforward(kS, kV);
        m_backLeftFF = new SimpleMotorFeedforward(kS, kV);
        m_backRightFF = new SimpleMotorFeedforward(kS, kV);
    }

    /**
     * Simultaneously updates all four motor feedforward kS, kV, and kA values.
     * @param kS The desired kS value.
     * @param kV The desired k value.
     * @param kA The desired kA value.
     */
    protected void resetFeedForward(double kS, double kV, double kA) {
        m_frontLeftFF = new SimpleMotorFeedforward(kS, kV, kA);
        m_frontRightFF = new SimpleMotorFeedforward(kS, kV, kA);
        m_backLeftFF = new SimpleMotorFeedforward(kS, kV, kA);
        m_backRightFF = new SimpleMotorFeedforward(kS, kV, kA);
    }
}