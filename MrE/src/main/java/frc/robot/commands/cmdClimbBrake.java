package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ClimbStage1Subsystem;

public class cmdClimbBrake extends CommandBase {

    private final ClimbStage1Subsystem m_climbStage1Subsystem;


    // private boolean isRunning = false;

    public cmdClimbBrake(ClimbStage1Subsystem subsystem) {


        m_climbStage1Subsystem = subsystem;
        addRequirements(m_climbStage1Subsystem);

    }

    // Called once when this command runs
    @Override
    public void initialize() {
        m_climbStage1Subsystem.setBrake();
    }
    
    @Override
    public void execute() {
        m_climbStage1Subsystem.setBrake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
