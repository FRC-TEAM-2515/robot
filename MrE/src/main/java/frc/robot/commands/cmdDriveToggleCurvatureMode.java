package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class cmdDriveToggleCurvatureMode extends InstantCommand {

    private final DriveSubsystem m_driveSubsystem;

    public cmdDriveToggleCurvatureMode(DriveSubsystem subsystem) {

        m_driveSubsystem = subsystem;
        addRequirements(m_driveSubsystem);

    }

    // Called once when this command runs
    @Override
    public void initialize() {
        m_driveSubsystem.toggleCurvatureMode();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
