package frc.robot.subsystems;

//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.HopperConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class HopperSubsystem extends SubsystemBase {
 
    private WPI_TalonSRX m_hopperStage1Leader;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private Color detectedColor;

    public HopperSubsystem() {
        m_hopperStage1Leader = new WPI_TalonSRX(5);
        m_hopperStage1Leader.setNeutralMode(NeutralMode.Brake);
        
    }

    @Override
    public void periodic() {
        detectedColor = m_colorSensor.getColor();
        // double IR = m_colorSensor.getIR();
        // int proximity = m_colorSensor.getProximity();
        SmartDashboard.putBoolean("Hopper Loaded", isLoaded());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void setOutput(double output) {
        m_hopperStage1Leader.set(output);
    }

    public void stopMotors() {
        m_hopperStage1Leader.set(0.0);
    }

    public boolean isRunning() {
        if (Math.abs(m_hopperStage1Leader.getMotorOutputPercent()) > 0.0) {

            return true;
        }
        return false;
    }

    public boolean isLoaded() {
        if (m_colorSensor.getProximity() > HopperConstants.kLoadedProximity) {
            RobotContainer.getInstance().m_intakeSubsystem.setOutput(0);
            RobotContainer.getInstance().m_VisionSubsystem.setLEDMode(0);
            return true;
    
        }else{
       // RobotContainer.getInstance().m_intakeSubsystem.setOutput(1);
        RobotContainer.getInstance().m_VisionSubsystem.setLEDMode(1);
        return false;
        }
    }

    
    public boolean isCorrectColor() {
        return false;
    }
}

