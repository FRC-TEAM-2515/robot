package frc.robot.subsystems;

// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class IntakeSubsystem extends SubsystemBase {
    private WPI_TalonSRX m_intake;

    public IntakeSubsystem() {
        m_intake = new WPI_TalonSRX(6);
        m_intake.setInverted(true);
        m_intake.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void setOutput(double output) {
        m_intake.set(output);
    }

    public void stopMotors(){
        m_intake.set(0.0);
    }

    public boolean isRunning(){
        if(Math.abs(m_intake.getMotorOutputPercent()) > 0.0){
            return true;
        }
        return false;
     }
}
