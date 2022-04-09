package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class cmdShooterToggle extends InstantCommand {

    private final ShooterSubsystem m_shooterSubsystem;

    private double output;

    public cmdShooterToggle(double output, ShooterSubsystem subsystem) {

        m_shooterSubsystem = subsystem;
        this.output = output;
        addRequirements(m_shooterSubsystem);

    }

    // Called once when this command runs
    @Override
    public void initialize() {
            m_shooterSubsystem.setOutput(output);
    }


    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
