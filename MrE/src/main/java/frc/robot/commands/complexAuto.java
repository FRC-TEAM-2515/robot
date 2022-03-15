// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class complexAuto extends SequentialCommandGroup {
  /** Creates a new complexDrive. */
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final HopperSubsystem m_hopperSubsystem;
  private final IntakeDeploySubsystem m_intakeDeploySubsystem;
  private double distance;

  public complexAuto(DriveSubsystem driveSubsystem,
                    ShooterSubsystem shooterSubsystem, 
                    HopperSubsystem hopperSubsystem, 
                    IntakeDeploySubsystem IntakeDeploySubsystem, 
                    Double distance) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_driveSubsystem = driveSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_hopperSubsystem = hopperSubsystem;
    this.m_intakeDeploySubsystem = IntakeDeploySubsystem;
    this.distance = m_driveSubsystem.getAverageEncoderDistance() + distance;
    addRequirements(m_driveSubsystem);
    addCommands(
      new autoDistanceDrive(m_driveSubsystem, DriveConstants.kAutoDriveDistance),
      new cmdIntakeDeploy(m_intakeDeploySubsystem, IntakeConstants.kMaxEncoderPosition),
      new cmdShooterSetRPM(ShooterConstants.kAutonomousRPM,m_shooterSubsystem),
      new cmdHopperToggle(m_hopperSubsystem)
    );
  }
}
