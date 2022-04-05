// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  static double distanceFromLimelightToGoalInches;
  static double angleToGoalRadians;
  static double angleToGoalDegrees;
  private static double targetOffsetAngle_Vertical;
  private static NetworkTable table;

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Target Found", isTargetFound());
    SmartDashboard.putNumber("Horizontal Distance", getDistance());
  }

  public static double getDistance(){
    // calculate horizontal distance
    targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0.0);
    angleToGoalDegrees = VisionConstants.kLimelightMountAngleDegrees + targetOffsetAngle_Vertical;
    angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
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
