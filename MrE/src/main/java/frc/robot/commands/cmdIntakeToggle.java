package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;

public class cmdIntakeToggle extends InstantCommand {

    private final IntakeSubsystem m_intakeSubsystem;

    public cmdIntakeToggle(IntakeSubsystem subsystem) {

        m_intakeSubsystem = subsystem;
        addRequirements(m_intakeSubsystem);
    }

    // Called once when this command runs
    @Override
    public void initialize() {
        if (m_intakeSubsystem.isRunning()) {
            m_intakeSubsystem.setOutput(0.0);
        } else {
            m_intakeSubsystem.setOutput(0.55);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
