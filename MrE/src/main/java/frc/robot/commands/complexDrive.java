// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class complexDrive extends SequentialCommandGroup {
  /** Creates a new complexDrive. */
  private final DriveSubsystem m_driveSubsystem;
  private double distance;
  public complexDrive(DriveSubsystem subsystem, Double distance) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_driveSubsystem = subsystem;
    this.distance = m_driveSubsystem.getAverageEncoderDistance() + distance;
    addRequirements(m_driveSubsystem);
    addCommands(new autoDistanceDrive(m_driveSubsystem, DriveConstants.kAutoDriveDistance));
  }
}
