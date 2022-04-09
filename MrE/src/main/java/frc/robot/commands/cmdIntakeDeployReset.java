package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeDeploySubsystem;

public class cmdIntakeDeployReset extends CommandBase {
  /** Creates a new cmdIntakeDeployReset. */
  private final IntakeDeploySubsystem m_intakeDeploySubsystem;
  
  public cmdIntakeDeployReset(IntakeDeploySubsystem subsystem) {
    this.m_intakeDeploySubsystem = subsystem;
  }

  @Override
  public void initialize() {
    m_intakeDeploySubsystem.resetEncoder();
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
