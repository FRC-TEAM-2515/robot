package frc.robot.subsystems;

import frc.robot.RobotContainer;
// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
// import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
// import edu.wpi.first.wpilibj.Encoder;

public class ClimbStage1Subsystem extends SubsystemBase {
    private WPI_TalonSRX m_climbMotorLeader;
    private WPI_VictorSPX m_climbMotorFollower;

    public ClimbStage1Subsystem() {
        m_climbMotorLeader = new WPI_TalonSRX(2);
        m_climbMotorFollower = new WPI_VictorSPX(22);
        m_climbMotorLeader.setNeutralMode(NeutralMode.Coast);
        m_climbMotorFollower.setNeutralMode(NeutralMode.Coast);
        m_climbMotorFollower.setInverted(true);

    }

    @Override
    public void periodic() {
        setOutput();
    }

    @Override
    public void simulationPeriodic() {

    }

    public void setOutput() {
        double output = applyDeadband(RobotContainer.getInstance().getcontrollerOperator().getLeftY());
        m_climbMotorLeader.set(output);
        m_climbMotorFollower.set(output);
    }

    public void setBrake() {
        m_climbMotorLeader.set(0.3);
        m_climbMotorFollower.set(0.3);
        //Negative if climb cable is wound underneath and vice versa
    }

    public void stopMotors() {
        m_climbMotorLeader.set(0.0);
        m_climbMotorFollower.set(0.0);
        m_climbMotorFollower.setNeutralMode(NeutralMode.Brake);
        m_climbMotorLeader.setNeutralMode(NeutralMode.Brake);
    }

    protected double applyDeadband(double value){
        double deadBand = 0.15;
        if(Math.abs(value) > deadBand){
            if(value > 0.0){
                return (value - deadBand) * (value - deadBand);
            } else{
                return -1 * (value + deadBand) *(value + deadBand);
            }
        } else {
            return 0.0;
        }
    }
}
