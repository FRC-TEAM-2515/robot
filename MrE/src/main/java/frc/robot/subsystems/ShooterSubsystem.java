package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.VisionConstants;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.controller.PIDController;

// import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.VisionSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_shooterLeader;
    // private CANSparkMax m_shooterFollower;
    // private MotorControllerGroup m_shooter;
    private RelativeEncoder m_encoderShooter;
    public SparkMaxPIDController m_pidController;

    // Initialize your subsystem here
    public ShooterSubsystem() {
        m_shooterLeader = new CANSparkMax(ShooterConstants.kShooterLeaderID, MotorType.kBrushless);

        m_shooterLeader.restoreFactoryDefaults();
        m_shooterLeader.setIdleMode(IdleMode.kCoast);

        m_pidController = m_shooterLeader.getPIDController();

        m_encoderShooter = m_shooterLeader.getEncoder();
       
        m_pidController.setP(ShooterConstants.kP);
        m_pidController.setI(ShooterConstants.kI);
        m_pidController.setD(ShooterConstants.kD);
        m_pidController.setIZone(ShooterConstants.kIz);
        m_pidController.setFF(ShooterConstants.kFF);
        m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    }

    @Override
    public void periodic() {
       
        SmartDashboard.putNumber("Shooter Velocity", m_encoderShooter.getVelocity());
        double o = SmartDashboard.getNumber("Shooter output", 0);
        if ((o != ShooterConstants.kShooterPercentOutput)) {
            ShooterConstants.kShooterPercentOutput = o;
        }

        SmartDashboard.putNumber("Target Output", getOutputFromVision());
       }

    

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

   
    public void stopMotors() {
        m_shooterLeader.set(0);
    }

    public void setVelocity(double setpoint) {
        setpoint = setpoint * ShooterConstants.kMaxRPM;
        m_pidController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
    }

    public void setOutput(double output) {
        m_shooterLeader.set(output);
    }

    public boolean isRunning() {
        if (Math.abs(m_shooterLeader.getOutputCurrent()) > 0.0) {
            return true;
        }
        return false;
    }

    public double getOutputFromVision() {
        double output;
        double distance = VisionSubsystem.getDistance();
        if (distance < 66) {
            output = 0.50;
        } else if (distance > 144) {
            output = .75;
        } else {
            output = 0.0000000004 * Math.pow(distance, 4) - 0.000000554 * Math.pow(distance, 3)
                    + 0.0001437401 * Math.pow(distance, 2) - 0.0112573212 * distance + 0.7682;
        }
        return output;
    }

    public boolean atSetpoint() {

        return false;
    }
}
