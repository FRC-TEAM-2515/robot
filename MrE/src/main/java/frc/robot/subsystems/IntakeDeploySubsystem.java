package frc.robot.subsystems;

// import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;

public class IntakeDeploySubsystem extends PIDSubsystem {

    private Encoder m_encoderIntakeDeploy;
    private WPI_TalonSRX m_intakeDeploy;
    private static final double kP = 0.02;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    // Initialize your subsystem here
    public IntakeDeploySubsystem() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(0.2);

        m_encoderIntakeDeploy = new Encoder(0, 1, false, EncodingType.k4X);
        addChild("m_encoderIntakeDeploy", m_encoderIntakeDeploy);
        m_encoderIntakeDeploy.setDistancePerPulse(1.0);
        m_encoderIntakeDeploy.setReverseDirection(true);

        m_intakeDeploy = new WPI_TalonSRX(4);
        m_intakeDeploy.setNeutralMode(NeutralMode.Brake);
        m_intakeDeploy.setInverted(true);
    
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Intake Position", m_encoderIntakeDeploy.getDistance());

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public double getMeasurement() {
        return m_encoderIntakeDeploy.getDistance();

    }

    @Override
    public void useOutput(double output, double setpoint) {
        output += setpoint * kF;
        m_intakeDeploy.set(output);

    }
    public void resetEncoder(){
    
        m_encoderIntakeDeploy.reset();
    }

    public void stopMotors() {
        m_intakeDeploy.set(0.0);
    }

    public void setBrake() {
        m_intakeDeploy.setNeutralMode(NeutralMode.Brake);
    }
    public void setCoast() {
        m_intakeDeploy.setNeutralMode(NeutralMode.Coast);
    }

    public double getPosition() {
        return m_encoderIntakeDeploy.getDistance();
    }
}
