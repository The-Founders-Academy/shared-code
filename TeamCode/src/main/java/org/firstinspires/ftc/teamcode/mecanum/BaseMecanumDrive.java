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


public abstract class BaseMecanumDrive extends SubsystemBase {

    public enum Alliance {
        RED, BLUE
    }

    protected SimpleMotorFeedforward m_frontLeftFF, m_frontRightFF, m_backLeftFF, m_backRightFF;
    protected MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    protected MecanumDriveKinematics m_kinematics;
    protected MecanumConfigs m_mecanumConfigs;
    protected Alliance m_alliance;

    public abstract Rotation2d getHeading();
    public abstract Pose2d getPose();
    public abstract void resetPose(Pose2d pose);


      /*
    This is how the PID loop will look in pseudocode.

    ... set PID setpoints up here

    setTargetPose(somePose);
    while not atTargetPose():
        moveWithPID();
        tunePIDs();
    resetPIDs();

     */
    public abstract void setTargetPose(Pose2d pose); // Set target translation/rotation in field relative coordinates
    public abstract boolean atTargetPose(); // Returns whether the robot is within margin of error of target
    public abstract void moveWithPID(); // Move field relative (not driver relative) towards the setpoint
    public abstract void resetPIDs(); // Reset the setpoints of each PID
    public abstract void tunePIDs(); // Retrieve PID values from dashboard
    protected PIDController m_translationXController;
    protected PIDController m_translationYController;
    protected PIDController m_rotationController;


    public BaseMecanumDrive(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose, Alliance alliance) {
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
        m_frontLeft.set(m_frontLeftFF.calculate(wheelSpeeds.frontLeftMetersPerSecond));
        m_frontRight.set(m_frontRightFF.calculate(wheelSpeeds.frontRightMetersPerSecond));
        m_backLeft.set(m_backLeftFF.calculate(wheelSpeeds.rearLeftMetersPerSecond));
        m_backRight.set(m_backRightFF.calculate(wheelSpeeds.rearRightMetersPerSecond));

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
}
