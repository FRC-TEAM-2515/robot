package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;

// import java.io.IOException;
// import java.nio.file.Path;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Ultrasonic;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    // public static String trajectoryBL1P1 = "paths/output/BL1P1.wpilib.json";
    // public static String trajectoryRL1P1 = "paths/output/RL1P1.wpilib.json";
    // public static String trajectoryDefault = "paths/output/Default.wpilib.json";

    public AnalogPotentiometer rangeFinder;

    @Override
    public void robotInit() {
        m_robotContainer = m_robotContainer.getInstance();
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
        m_robotContainer.m_driveSubsystem.resetEncoders();
        m_robotContainer.m_driveSubsystem.resetGyro();
        m_robotContainer.m_intakeDeploySubsystem.resetEncoder();
        //CameraServer.startAutomaticCapture();
        rangeFinder = new AnalogPotentiometer(0, 180, 30);

    //     try {
    //         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryBL1P1);
    //         m_robotContainer.m_trajectoryBL1P1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //     } catch (IOException ex) {
    //         DriverStation.reportError("Unable to open trajectory: " + trajectoryBL1P1, ex.getStackTrace());
    //     }
    //     try {
    //         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryRL1P1);
    //         m_robotContainer.m_trajectoryRL1P1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //     } catch (IOException ex) {
    //         DriverStation.reportError("Unable to open trajectory: " + trajectoryBL1P1, ex.getStackTrace());
    //     }
    //     try {
    //         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryDefault);
    //         m_robotContainer.m_trajectoryDefault = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //     } catch (IOException ex) {
    //         DriverStation.reportError("Unable to open trajectory: " + trajectoryBL1P1, ex.getStackTrace());
    //     }
    }

   
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.safeReset();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_robotContainer.m_driveSubsystem.resetEncoders();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

   
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        m_robotContainer.deployIntake();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.m_driveSubsystem.pilotDrive();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

}
