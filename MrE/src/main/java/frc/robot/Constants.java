// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final class DriveConstants {

        public static final int kDriveRightLeaderID = 13;
        public static final int kDriveRigthFollowerID = 14;
        public static final int kDriveLeftLeaderID = 11;
        public static final int kDriveLeftFollowerID = 12;

        public static final double kDeadband = 0.07;
        public static final double kAcellerationLimit = 0.8; // Slew rate limiter
        public static final double kSteeringOutputModifier = 0.6; //Turn speed

        public static final double kWheelDiameterMeter = 0.1524;
        public static final double kWheelDiameterInches = 6.0;
        public static final double kDriveGearRatio = 10.71;
        public static final double kEncoderTicksPerRevolution = 168; // 96 tested, actual from documentation is 42;
        public static final double kEncoderTick2Feet = 1.0 / kEncoderTicksPerRevolution * kDriveGearRatio
                * kWheelDiameterInches * Math.PI / 12;
        public static final double kEncoderTick2Meter = 1.0 / kEncoderTicksPerRevolution * kDriveGearRatio
                * kWheelDiameterMeter * Math.PI;

        public static final double kEncoderPositionConversionFactor = kEncoderTick2Meter;
        public static final double kEncoderVelocityConversionFactor = kDriveGearRatio * Math.PI * kWheelDiameterMeter
                / 60;

        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 5;
        public static final double kAutoDriveDistance = -1; //Meters


        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.11653;
        public static final double kvVoltSecondsPerMeter = 0.12895;
        public static final double kaVoltSecondsSquaredPerMeter = 0.016919;
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.5;

        public static final double kTrackwidthMeters = 0.5588;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class ShooterConstants {
        public static final double kVelocityConversionFactor = 1.0;
        public static int kShooterLeaderID = 19;
        public static int kShooterFollowerID = 18;
        public static double kShooterPercentOutput = 0.55;
        public static double kMaxRPM = 5700;
        public static double kMinRPM = 2500;
        public static double kAutonomousSetpoint = 2000;
        public static double kIz = 0;
        public static double kFF = 0.000015;
        public static double kMinOutput = 0;
        public static double kMaxOutput = 1;
        public static double kP = 6e-5;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static final double kF = 0.000015;
        public static final double kSpinupDelay = 1;

        // From example code.
        // kP = 6e-5; 
        // kI = 0;
        // kD = 0; 
        // kIz = 0; 
        // kFF = 0.000015; 
        // kMaxOutput = 1; 
        // kMinOutput = -1;
        // maxRPM = 5700;
    }

    public static final class IntakeConstants {
            public static final double kMaxEncoderPosition = 130.0; //144 full deployed
    }

    public static final class HopperConstants {
            public static final double kLoadedProximity = 100;
    }
}
