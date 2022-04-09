package frc.robot.commands;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class cmdToggleVision extends CommandBase {
  /** Creates a new cmdToggleVision. */
  //  private final VisionSubsystem m_visionSubsystem;
  public cmdToggleVision(VisionSubsystem subsystem) {
    //  this.m_visionSubsystem = subsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double limelightLEDMode = table.getEntry("ledMode").getDouble(0);
    if (limelightLEDMode == 1){
      table.getEntry("<variablename>").setNumber(3);
    } else {
      table.getEntry("<variablename>").setNumber(1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
