// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.stuypulse.stuylib.input.gamepads.*;
import com.stuypulse.stuylib.input.Gamepad;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

/**
 * COPIED FROM PRACTICE BOT
 * MOST COMMENTS ARE BECAUSE I WAS TOO TIRED TO FIX ANYTHING BEYOND CTRL F AND REPLACE
 * 
 * NEEDS TO BE UPDATED FOR MOTION, PID, & ALTERNATIVE DRIVE CONTROLS
 * 
 * @author 23bbrenner
 */
public class DriveTrain extends SubsystemBase {

    // Constants
    static final double m_turnGain = DriveConstants.kTurnGain;
    static final double m_deadband = DriveConstants.kDeadband;
    static final double m_driveGain = DriveConstants.kDriveGain;

    // Hardware
    private WPI_TalonSRX m_driveLeftLeader;
    private WPI_VictorSPX m_driveLeftFollower;
    private WPI_TalonSRX m_driveRightLeader;
    private WPI_VictorSPX m_driveRightFollower;
    private AHRS gyro;

    private DifferentialDrive m_drive;
    private MotorControllerGroup m_driveLeft;
    private MotorControllerGroup m_driveRight;
    private DifferentialDriveOdometry odometry;

    // Controllers
    protected Gamepad driveController;
    

    /**
    *
    */
    public DriveTrain() {

    m_driveLeftLeader = new WPI_TalonSRX(DriveConstants.kLeftLeaderPort);
    m_driveLeftLeader.configFactoryDefault();
    m_driveLeftFollower = new WPI_VictorSPX(DriveConstants.kLeftFollowerPort);
    m_driveLeftFollower.configFactoryDefault();

    m_driveRightLeader = new WPI_TalonSRX(DriveConstants.kRightLeaderPort);
    m_driveRightLeader.configFactoryDefault();
    m_driveRightFollower = new WPI_VictorSPX(DriveConstants.kRightFollowerPort);
    m_driveRightFollower.configFactoryDefault();
   
    // Current Limit - prevents breakers from tripping in PDP
    //!!arbitary values, need update!!
    //  m_driveLeftLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    //  m_driveRightLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));


    /* Set the peak and nominal outputs */
    m_driveLeftLeader.configNominalOutputForward(0, 30);
    m_driveLeftLeader.configNominalOutputReverse(0, 30);
    m_driveLeftLeader.configPeakOutputForward(1, 30);
    m_driveLeftLeader.configPeakOutputReverse(-1, 30);
    
    m_driveRightLeader.configNominalOutputForward(0, 30);
    m_driveRightLeader.configNominalOutputReverse(0, 30);
    m_driveRightLeader.configPeakOutputForward(1, 30);
    m_driveRightLeader.configPeakOutputReverse(-1, 30);

    
    /* Invert Motor? and set Break Mode */
    m_driveLeftLeader.setInverted(false);
    m_driveLeftLeader.setNeutralMode(NeutralMode.Coast);
    m_driveLeftFollower.setNeutralMode(NeutralMode.Coast);


    m_driveRightLeader.setInverted(true);
    m_driveRightFollower.setInverted(true);
    m_driveRightLeader.setNeutralMode(NeutralMode.Coast);
    m_driveRightFollower.setNeutralMode(NeutralMode.Coast);
    
    /* Configure Sensor */
    // Phase sensor to have positive increment when driving Talon Forward (Green
    // LED)
    m_driveLeftLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    m_driveLeftLeader.setSensorPhase(false);

    m_driveRightLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    m_driveRightLeader.setSensorPhase(true);

    
    m_driveLeftFollower.follow(m_driveLeftLeader);
    m_driveRightFollower.follow(m_driveRightLeader);


    /* PID Values */
    //Need to add closed loop/motion profiling for them to be relevant

    m_driveLeftLeader.selectProfileSlot(0, 0);
    m_driveLeftLeader.config_kF(0, 0.0, 30);
    m_driveLeftLeader.config_kP(0, 0.0, 30);
    m_driveLeftLeader.config_kI(0, 0.0, 30);
    m_driveLeftLeader.config_kD(0, 0.0, 30);

    m_driveRightLeader.selectProfileSlot(0, 0);
    m_driveRightLeader.config_kF(0, 0.0, 30);
    m_driveRightLeader.config_kP(0, 0.0, 30);
    m_driveRightLeader.config_kI(0, 0.0, 30);
    m_driveRightLeader.config_kD(0, 0.0, 30);

    
    m_driveLeft = new MotorControllerGroup(m_driveLeftLeader, m_driveLeftFollower);
    m_driveRight = new MotorControllerGroup(m_driveRightLeader, m_driveRightFollower);
    m_drive = new DifferentialDrive(m_driveLeft, m_driveRight);

    m_drive.setSafetyEnabled(true);
    m_drive.setExpiration(0.1);
    m_drive.setMaxOutput(1.0);

    try {
        gyro = new AHRS(Port.kMXP);
    } catch (RuntimeException ex) {
        DriverStation.reportError(ex.getMessage(), true);
    }
    Timer.delay(1.0);
    
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), m_driveGain, m_deadband);
}
    

@Override
public void periodic() {
    // This method will be called once per scheduler run
}

@Override
public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

}

// Put methods for controlling this subsystem
// here. Call these from Commands.

//** RECONFIG **/

public void manualDrive() {
    if (driveController == null) {
        driveController = RobotContainer.getInstance().getDriveController();
    }
}
//     if (curvatureDriveMode) {
//         if (m_driveRightStickEnabled) {
//             m_drive.curvatureDrive(applyGain(driveController.getLeftY(), driveGain),
//                     applyGain(driveController.getRightX(), turnGain), driveController.getRawButton(10));
//         } else {
//             m_drive.curvatureDrive(applyGain(driveController.getLeftY(), driveGain),
//                     applyGain(driveController.getLeftX(), turnGain), driveController.getAButton());
//         }
//     } else {
//         if (m_driveRightStickEnabled) {
//             m_drive.arcadeDrive(applyGain(driveController.getLeftY(), driveGain),
//                     applyGain(driveController.getRightX(), turnGain));
//         } else {
//             m_drive.arcadeDrive(applyGain(driveController.getLeftY(), driveGain),
//                     applyGain(driveController.getLeftX(), turnGain));
//         }
//     }
// }

// private double applyGain(double x, double gain) {
//     if (x > -m_deadband && x < m_deadband) {
//         x = 0;
//     } else if (x >= m_deadband) {
//         x = x - m_deadband;
//         x = x / (1 - m_deadband);
//         x = Math.pow(x, gain);
//         x = -x;
//     } else {
//         x = x + m_deadband;
//         x = x / (1 - m_deadband);
//         x = Math.pow(x, gain);
//     }
//     return x;
// }

public void updateSmartDashboard(){
    SmartDashboard.putNumber("imu-yaw", gyro.getYaw());
    SmartDashboard.putNumber("imu-pitch", gyro.getPitch());
    SmartDashboard.putNumber("imu-roll", gyro.getRoll());
    SmartDashboard.putNumber("imu-angle", gyro.getAngle());

    SmartDashboard.putBoolean("imu-moving", gyro.isMoving());
    SmartDashboard.putBoolean("imu-connected", gyro.isConnected());
    SmartDashboard.putBoolean("imu-calibrating", gyro.isCalibrating());
    // SmartDashboard.putData("imu", gyro);

    SmartDashboard.putNumber("Left Velocity", m_driveLeftLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Velocity", m_driveRightLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Distance", m_driveLeftLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Distance", m_driveRightLeader.getSelectedSensorPosition());

}
/**
 * Returns the currently-estimated pose of the robot.
 *
 * @return The pose.
 */
public Pose2d getPose() {
    return odometry.getPoseMeters();
}

/**
 * Returns the current wheel speeds of the robot.
 *
 * @return The current wheel speeds.
 */
public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // Convert RPMs to Meters per Second
    return new DifferentialDriveWheelSpeeds(
            m_driveLeftLeader.getSelectedSensorVelocity(),
            m_driveRightLeader.getSelectedSensorVelocity());
}

/**
 * Resets the odometry to the specified pose.
 *
 * @param pose The pose to which to set the odometry.
 */
public void resetOdometry(Pose2d pose) {
    resetEncoders();
    //odometry.resetPosition(gyro.getRotation2d(),m_driveLeftLeader.getSelectedSensorPosition(),m_driveRightLeader.getSelectedSensorPosition(),pose);
}

/**
 * Drives the robot using arcade controls.
 *
 * @param fwd the commanded forward movement
 * @param rot the commanded rotation
 */
public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
}

/**
 * Controls the m_driveLeft and m_driveRight sides of the drive directly with voltages.
 *
 * @param m_driveLeftVolts  the commanded m_driveLeft output
 * @param m_driveRightVolts the commanded m_driveRight output
 */
public void tankDriveVolts(double m_driveLeftVolts, double m_driveRightVolts) {
    m_driveLeftLeader.setVoltage(m_driveLeftVolts);
    m_driveRightLeader.setVoltage(m_driveRightVolts);
    m_drive.feed();
}

/** Resets the drive encoders to currently read a position of 0. */
public void resetEncoders() {
    m_driveLeftLeader.setSelectedSensorPosition(0);
    m_driveRightLeader.setSelectedSensorPosition(0);
}

/**
 * Gets the average distance of the two encoders.
 *
 * @return the average of the two encoder readings
 */
public double getAverageEncoderDistance() {
    return (m_driveLeftLeader.getSelectedSensorPosition() + m_driveRightLeader.getSelectedSensorPosition() / 2.0);
}

// /**
//  * Gets the m_driveLeft drive encoder.
//  *
//  * @return the m_driveLeft drive encoder
//  */
// public RelativeEncoder getLeftEncoder() {
//     return m_driveEncoderLeft1;
// }

// /**
//  * Gets the m_driveRight drive encoder.
//  *
//  * @return the m_driveRight drive encoder
//  */
// public RelativeEncoder getRightEncoder() {
//     return m_driveEncoderRight1;
// }

/**
 * Sets the max output of the drive. Useful for scaling the drive to drive more
 * slowly.
 *
 * @param maxOutput the maximum output to which the drive will be constrained
 */
public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
}

/** Zeroes the heading of the robot. */
public void zeroHeading() {
    gyro.reset();
}

/**
 * Returns the heading of the robot.
 *
 * @return the robot's heading in degrees, from -180 to 180
 */
public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
}

/**
 * Returns the turn rate of the robot.
 *
 * @return The turn rate of the robot, in degrees per second
 */
public double getTurnRate() {
    return -gyro.getRate();
}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}
