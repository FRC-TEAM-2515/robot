package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
import frc.robot.subsystems.ShooterSubsystem;

public class cmdShooterSetRPM extends CommandBase {

    private final ShooterSubsystem m_shooterSubsystem;
    private double m_setpoint;

    public cmdShooterSetRPM(ShooterSubsystem subsystem) {
        this(0, subsystem);
    }

    public cmdShooterSetRPM(double setpoint, ShooterSubsystem subsystem) {
        super();
        this.m_setpoint = setpoint;

        m_setpoint = setpoint;

        m_shooterSubsystem = subsystem;
        addRequirements(m_shooterSubsystem);

    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {;
        m_shooterSubsystem.setVelocity(m_setpoint);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // @Override
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {

    }

}
