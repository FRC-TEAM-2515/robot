package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class cmdDriveResetEncoders extends InstantCommand {

    private final DriveSubsystem m_driveSubsystem;

    public cmdDriveResetEncoders(DriveSubsystem subsystem) {

        m_driveSubsystem = subsystem;
        addRequirements(m_driveSubsystem);

    }

    // Called once when this command runs
    @Override
    public void initialize() {
        m_driveSubsystem.resetEncoders();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
