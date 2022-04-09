package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.RobotContainer;
import frc.robot.subsystems.HopperSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

public class cmdIsRobotSquare extends InstantCommand {

    private final HopperSubsystem m_hopperSubsystem;

    public cmdIsRobotSquare(HopperSubsystem subsystem) {

        m_hopperSubsystem = subsystem;
        addRequirements(m_hopperSubsystem);
    }

    // Called once when this command runs
    @Override
    public void initialize() {
    //    HopperSubsystem.isRobotSquare();
    }


    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
