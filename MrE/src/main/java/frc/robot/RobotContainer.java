package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.subsystems.*;


public class RobotContainer {

        public static RobotContainer m_robotContainer = new RobotContainer();

        // public static Alliance allianceColor = DriverStation.getAlliance();

        // public Trajectory m_trajectoryBL1P1 = new Trajectory();
        // public Trajectory m_trajectoryRL1P1 = new Trajectory();
        // public Trajectory m_trajectoryDefault = new Trajectory();

        // The robot's subsystems

        public final ClimbStage1Subsystem m_climbStage1Subsystem = new ClimbStage1Subsystem();
        public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
        public final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
        public final IntakeDeploySubsystem m_intakeDeploySubsystem = new IntakeDeploySubsystem();
        public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
        public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
        public final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

        // Joysticks
        private final XboxController controllerOperator = new XboxController(1);
        private final XboxController controllerPilot = new XboxController(0);

        // A chooser for autonomous commands
        SendableChooser<Command> m_chooser = new SendableChooser<>();
        // SendableChooser<Trajectory> m_trajectoryChooser = new SendableChooser<>();
       
        private RobotContainer() {
                configureButtonBindings();

                m_chooser.addOption("Autonomous Command", new AutonomousCommand());
                m_chooser.setDefaultOption("Complex Auto", new complexAuto(
                                m_driveSubsystem,
                                m_shooterSubsystem,
                                m_hopperSubsystem,
                                m_intakeDeploySubsystem,
                                DriveConstants.kAutoDriveDistance));
                m_chooser.addOption("Reverse Distance", 
                        new autoDistanceDrive(m_driveSubsystem, DriveConstants.kAutoDriveDistance));

                // if (allianceColor == DriverStation.Alliance.Blue) {
                //         m_trajectoryChooser.setDefaultOption("Blue Position 1", m_trajectoryBL1P1);
                //         m_trajectoryChooser.addOption("Red Position 1", m_trajectoryRL1P1);
                // } else if (allianceColor == DriverStation.Alliance.Red) {
                //         m_trajectoryChooser.setDefaultOption("Red Position 1", m_trajectoryRL1P1);
                //         m_trajectoryChooser.addOption("Blue Position 1", m_trajectoryBL1P1);
                // } else {
                //         m_trajectoryChooser.setDefaultOption("Default", m_trajectoryDefault);
                // }

                 SmartDashboard.putData("Auto Mode", m_chooser);
                // SmartDashboard.putData("Trajectory", m_trajectoryChooser);
        }

        public static RobotContainer getInstance() {
                return m_robotContainer;
        }


        private void configureButtonBindings() {

                SmartDashboard.putData("Reset Intake", new cmdIntakeDeployReset(m_intakeDeploySubsystem));
                // Create some buttons

                //  final JoystickButton btnOperatorClimbStage1Setpoint = new JoystickButton(controllerOperator,
                              //  XboxController.Button.kLeftStick.value);

                final JoystickButton btnOperatorClimbBrake = new JoystickButton(controllerOperator,
                                XboxController.Button.kLeftBumper.value);
                btnOperatorClimbBrake.whenPressed(new cmdClimbBrake(m_climbStage1Subsystem),
                                true);

                final JoystickButton btnShoot = new JoystickButton(controllerPilot,
                XboxController.Button.kRightBumper.value);
                btnShoot.whenPressed(new cmdShooterToggle(ShooterConstants.kMinOutput, m_shooterSubsystem),
                true);
                // SmartDashboard.putData("btnShoot", new cmdShoot(m_shooterSubsystem,
                // m_hopperSubsystem));

                final JoystickButton btnHopperToggle = new JoystickButton(controllerPilot,
                                XboxController.Button.kX.value);
                btnHopperToggle.whenPressed(new cmdHopperToggle(m_hopperSubsystem), true);
                SmartDashboard.putData("btnHopperToggle", new cmdHopperToggle(m_hopperSubsystem));

                final JoystickButton btnSafeRest = new JoystickButton(controllerPilot,
                                XboxController.Button.kLeftStick.value);
                btnSafeRest.whenPressed(new cmdSafeReset(), true);
                SmartDashboard.putData("btnSafeRest", new cmdSafeReset());

                final JoystickButton btnIntakeToggle = new JoystickButton(controllerPilot,
                                XboxController.Button.kLeftBumper.value);
                btnIntakeToggle.whenPressed(new cmdIntakeToggle(m_intakeSubsystem), true);
                SmartDashboard.putData("Intake Toggle", new cmdIntakeToggle(m_intakeSubsystem));

                final JoystickButton btnShooterSetRPMMin = new JoystickButton(controllerPilot,
                                XboxController.Button.kA.value);
                btnShooterSetRPMMin.whenPressed(new cmdShooterToggle(ShooterConstants.kMidOutput,
                                m_shooterSubsystem), true);

                final JoystickButton btnShooterToggle = new JoystickButton(controllerPilot,
                                XboxController.Button.kB.value);
                btnShooterToggle.whenPressed(new cmdShooterToggle(m_shooterSubsystem.getOutputFromVision(),
                                m_shooterSubsystem), true);

                final JoystickButton btnShooterSetRPMMax = new JoystickButton(controllerPilot,
                                XboxController.Button.kY.value);
                btnShooterSetRPMMax.whenPressed(new cmdShooterToggle(ShooterConstants.kMaxOutput,
                                m_shooterSubsystem), true);

                // final JoystickButton btnDriveToggleRightStickMode = new JoystickButton(controllerPilot,
                //                 XboxController.Button.kStart.value);
                // btnDriveToggleRightStickMode.whenPressed(new cmdDriveToggleRightStickMode(m_driveSubsystem), true);
                //      SmartDashboard.putData("btnDriveToggleRightStickMode",
                //                 new cmdDriveToggleRightStickMode(m_driveSubsystem));

                //  final JoystickButton btnTargetOrient = new JoystickButton(controllerPilot,
                //                   XboxController.Button.kBack.value);
                //  btnTargetOrient.whenPressed(new cmdTargetOrient(m_driveSubsystem), true);
                //  SmartDashboard.putData("Target Orient", new cmdTargetOrient(m_driveSubsystem));
              //   SmartDashboard.putBoolean("Is Target Orient",cmdTargetOrient.isTargetOrient());

                SmartDashboard.putData("Deploy Intake", new cmdIntakeDeploy(m_intakeDeploySubsystem, IntakeConstants.kMaxEncoderPosition));
                SmartDashboard.putNumber("Backup Distance", DriveConstants.kAutoDriveDistance);
                SmartDashboard.putNumber("Current Mid Output", ShooterConstants.kMidOutput);
                // SmartDashboard.putData("Is Robot Square", new cmdIsRobotSquare(m_hopperSubsystem));
                // SmartDashboard.putBoolean("Is Robot Square?", HopperSubsystem.isRobotSquare());  

                // SendableChooser m_isRobotSquare = new SendableChooser<>();

                // m_isRobotSquare.addOption("Yes", new cmdIsRobotSquare(m_hopperSubsystem));
                // m_isRobotSquare.addOption("No", new cmdHopperToggle(m_hopperSubsystem));

        }

        public XboxController getcontrollerPilot() {
                return controllerPilot;
        }

        public XboxController getcontrollerOperator() {
                return controllerOperator;
        }

        public Command getAutonomousCommand() {
                // The selected command will be run in autonomous
                return m_chooser.getSelected();

        
        }

        // public Trajectory getTrajectory() {
        //         if (m_trajectoryChooser.getSelected() != null) {
        //                 return m_trajectoryChooser.getSelected();
        //         }
        //         return m_trajectoryDefault;
        // }

        public void safeReset() {
                m_driveSubsystem.stopMotors();
                m_shooterSubsystem.stopMotors();
                m_climbStage1Subsystem.stopMotors();
                m_intakeSubsystem.stopMotors();
                m_intakeDeploySubsystem.stopMotors();
                m_hopperSubsystem.stopMotors();
                m_driveSubsystem.resetEncoders();
        }

        public void deployIntake() {
                new cmdIntakeDeploy(m_intakeDeploySubsystem, IntakeConstants.kMaxEncoderPosition);
                if (!m_intakeSubsystem.isRunning()) {
                        new cmdIntakeToggle(m_intakeSubsystem);
                }
        }

        // public static Alliance getAllianceColor() {
        //         return allianceColor;
        // }
}
