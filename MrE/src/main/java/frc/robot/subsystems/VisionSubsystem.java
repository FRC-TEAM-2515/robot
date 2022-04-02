// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  // private static final double limelightMountAngleDegrees = 0;
  static double distanceFromLimelightToGoalInches;
  // double goalHeightInches;
  double angleToGoalRadians;
  // double limelightLensHeightInches;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    //public double limelightMountAngleDegrees = 25.0;

    // distance from the center of the Limelight lens to the floor
    //limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    //goalHeightInches = 60.0;

    double angleToGoalDegrees = VisionConstants.kLimelightMountAngleDegrees + targetOffsetAngle_Vertical;
    angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // calculate horizontal distance
        distanceFromLimelightToGoalInches = (VisionConstants.kGoalHeightInches - VisionConstants.kLimelightLensHeightInches)
        / Math.tan(angleToGoalRadians);    
        SmartDashboard.putBoolean("Target Found", isTargetFound());
        SmartDashboard.putNumber("Horizontal Distance", getDistance());
  }

  public double getDistance(){
    distanceFromLimelightToGoalInches = (VisionConstants.kGoalHeightInches - VisionConstants.kLimelightLensHeightInches)
        / Math.tan(angleToGoalRadians); 
    return distanceFromLimelightToGoalInches;
  }

  public void setLEDMode(double mode){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
  }

  public boolean isTargetFound(){
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1){
      return true;
    }
    return false;
  } 
}
