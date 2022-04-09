package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
import frc.robot.subsystems.IntakeDeploySubsystem;

public class cmdIntakeDeploy extends CommandBase {

    private final IntakeDeploySubsystem m_intakeDeploySubsystem;
    private double m_setpoint;

    public cmdIntakeDeploy(IntakeDeploySubsystem subsystem, double setpoint) {
        super();
        this.m_setpoint = setpoint;

        m_setpoint = setpoint;


        m_intakeDeploySubsystem = subsystem;
        addRequirements(m_intakeDeploySubsystem);

    }

    @Override
    public void initialize() {
        m_intakeDeploySubsystem.enable();
        m_intakeDeploySubsystem.setSetpoint(m_setpoint);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (m_intakeDeploySubsystem.getPosition() > m_setpoint){
            m_intakeDeploySubsystem.setBrake();
            m_intakeDeploySubsystem.disable();
            m_intakeDeploySubsystem.setSetpoint(0);
            return true;
        }
        
        return m_intakeDeploySubsystem.getController().atSetpoint();

    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_intakeDeploySubsystem.stopMotors();
        m_intakeDeploySubsystem.setCoast();
    }

}
