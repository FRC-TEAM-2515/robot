package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class cmdHopperToggle extends InstantCommand {

    private final HopperSubsystem m_hopperSubsystem;

    public cmdHopperToggle(HopperSubsystem subsystem) {

        m_hopperSubsystem = subsystem;
        addRequirements(m_hopperSubsystem);

    }

    // Called once when this command runs
    @Override
    public void initialize() {
        if (m_hopperSubsystem.isRunning()  ) {
            m_hopperSubsystem.setOutput(0.0);
        } else {
            //RobotContainer.getInstance().m_driveSubsystem.setSteeringAdjustment();
            m_hopperSubsystem.setOutput(1);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
