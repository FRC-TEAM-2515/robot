// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

        public static RobotContainer m_robotContainer = new RobotContainer();

        public static Alliance allianceColor = DriverStation.getAlliance();

        public Trajectory m_trajectoryBL1P1 = new Trajectory();
        public Trajectory m_trajectoryRL1P1 = new Trajectory();
        public Trajectory m_trajectoryDefault = new Trajectory();

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
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

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

        // A chooser for autonomous commands
        SendableChooser<Command> m_chooser = new SendableChooser<>();
        SendableChooser<Trajectory> m_trajectoryChooser = new SendableChooser<>();
       
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        private RobotContainer() {
                // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
                // Smartdashboard Subsystems

                // SmartDashboard Buttons
                // SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
                // SmartDashboard.putData("Auto Distance Drive", new
                // autoDistanceDrive(m_driveSubsystem, DriveConstants.kAutoDriveDistance));
                // SmartDashboard.putData("cmdDriveToggleCurvatureMode",
                                // new cmdDriveToggleCurvatureMode(m_driveSubsystem));
                // SmartDashboard.putData("cmdDriveToggleRightStickMode", new
                // cmdDriveToggleRightStickMode());
                // SmartDashboard.putData("cmdShooterSetRPM: setpointMinRPM",
                // new cmdShooterSetRPM(200, m_shooterSubsystem));
                // SmartDashboard.putData("cmdShooterSetRPM: setpointMaxRPM",
                // new cmdShooterSetRPM(500, m_shooterSubsystem));
                // SmartDashboard.putData("cmdIntakeToggle", new cmdIntakeToggle(m_intakeSubsystem));
                // SmartDashboard.putData("cmdSafeReset", new cmdSafeReset());
                // SmartDashboard.putData("cmdHopperToggle", new cmdHopperToggle(m_hopperSubsystem));
                // SmartDashboard.putNumber("Shooter output", ShooterConstants.kShooterPercentOutput);

                // SmartDashboard.putData("cmdShoot", new cmdShoot(m_shooterSubsystem,
                // m_hopperSubsystem));

                // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

                // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

                // Configure autonomous sendable chooser
                // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

                m_chooser.addOption("Autonomous Command", new AutonomousCommand());
                m_chooser.setDefaultOption("Complex Auto", new complexAuto(
                                m_driveSubsystem,
                                m_shooterSubsystem,
                                m_hopperSubsystem,
                                m_intakeDeploySubsystem,
                                DriveConstants.kAutoDriveDistance));
                m_chooser.addOption("Reverse Distance", 
                        new autoDistanceDrive(m_driveSubsystem, DriveConstants.kAutoDriveDistance));

                // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

                if (allianceColor == DriverStation.Alliance.Blue) {
                        m_trajectoryChooser.setDefaultOption("Blue Position 1", m_trajectoryBL1P1);
                        m_trajectoryChooser.addOption("Red Position 1", m_trajectoryRL1P1);
                } else if (allianceColor == DriverStation.Alliance.Red) {
                        m_trajectoryChooser.setDefaultOption("Red Position 1", m_trajectoryRL1P1);
                        m_trajectoryChooser.addOption("Blue Position 1", m_trajectoryBL1P1);
                } else {
                        m_trajectoryChooser.setDefaultOption("Default", m_trajectoryDefault);
                }

                 SmartDashboard.putData("Auto Mode", m_chooser);
                // SmartDashboard.putData("Trajectory", m_trajectoryChooser);
        }

        public static RobotContainer getInstance() {
                return m_robotContainer;
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a
         * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                SmartDashboard.putData("Reset Intake", new cmdIntakeDeployReset(m_intakeDeploySubsystem));
                // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
                // Create some buttons

                final JoystickButton btnOperatorClimbStage1Setpoint = new JoystickButton(controllerOperator,
                                XboxController.Button.kLeftStick.value);

                final JoystickButton btnOperatorClimbBrake = new JoystickButton(controllerOperator,
                                XboxController.Button.kLeftBumper.value);
                btnOperatorClimbBrake.whenPressed(new cmdClimbBrake(m_climbStage1Subsystem),
                                true);

                // final JoystickButton btnOperatorHopperToggle = new
                // JoystickButton(controllerOperator,
                // XboxController.Button.kX.value);
                // btnOperatorHopperToggle.whenPressed(new cmdHopperToggle(m_hopperSubsystem),
                // true);
                // SmartDashboard.putData("btnOperatorHopperToggle", new
                // cmdHopperToggle(m_hopperSubsystem));

                // final JoystickButton btnOperatorShooterSetRPMMin = new
                // JoystickButton(controllerOperator,
                // XboxController.Button.kA.value);
                // btnOperatorShooterSetRPMMin.whenPressed(new cmdShooterSetRPM(200,
                // m_shooterSubsystem), true);
                // SmartDashboard.putData("btnOperatorShooterSetRPMMin", new
                // cmdShooterSetRPM(200, m_shooterSubsystem));

                // final JoystickButton btnOperatorShooterSetRPMMax = new
                // JoystickButton(controllerOperator,
                // XboxController.Button.kY.value);
                // btnOperatorShooterSetRPMMax.whenPressed(new cmdShooterSetRPM(500,
                // m_shooterSubsystem), true);
                // SmartDashboard.putData("btnOperatorShooterSetRPMMax", new
                // cmdShooterSetRPM(500, m_shooterSubsystem));

                // final JoystickButton btnOperatorIntakeToggle = new
                // JoystickButton(controllerOperator,
                // XboxController.Button.kLeftBumper.value);
                // btnOperatorIntakeToggle.whenPressed(new cmdIntakeToggle(m_intakeSubsystem),
                // true);
                // SmartDashboard.putData("Intake Toggle", new
                // cmdIntakeToggle(m_intakeSubsystem));

                // final JoystickButton btnOperatorShooterToggle = new
                // JoystickButton(controllerOperator,
                // XboxController.Button.kB.value);
                // btnOperatorShooterToggle.whenPressed(new
                // cmdShooterToggle(m_shooterSubsystem), true);

                // final JoystickButton btnOperatorHopperToggle = new
                // JoystickButton(controllerOperator,
                // XboxController.Button.kX.value);
                // btnOperatorHopperToggle.whenPressed(new cmdHopperToggle(m_hopperSubsystem),
                // true);
                // SmartDashboard.putData("btnHopperToggle", new
                // cmdHopperToggle(m_hopperSubsystem));

                // final JoystickButton btnOperatorShooterSetRPMMin = new
                // JoystickButton(controllerOperator,
                // XboxController.Button.kA.value);
                // btnOperatorShooterSetRPMMin.whenPressed(new
                // cmdShooterSetRPM(ShooterConstants.kMinRPM,
                // m_shooterSubsystem), true);

                // SmartDashboard.putData("btnOperatorIntakeToggle", new
                // cmdIntakeToggle(m_intakeSubsystem));

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

                final JoystickButton btnDriveToggleRightStickMode = new JoystickButton(controllerPilot,
                                XboxController.Button.kStart.value);
                btnDriveToggleRightStickMode.whenPressed(new cmdDriveToggleRightStickMode(m_driveSubsystem), true);
                // SmartDashboard.putData("btnDriveToggleRightStickMode",
                //                 new cmdDriveToggleRightStickMode(m_driveSubsystem));

                // final JoystickButton btnDriveToggleCurvatureMode = new JoystickButton(controllerPilot,
                //               XboxController.Button.kBack.value);
                // btnDriveToggleCurvatureMode.whenPressed(new cmdDriveToggleCurvatureMode(m_driveSubsystem), true);
                // SmartDashboard.putData("btnDriveToggleCurvatureMode",
                //                 new cmdDriveToggleCurvatureMode(m_driveSubsystem));
                // SmartDashboard.putData("Reset Drive Encoders", new cmdDriveResetEncoders(m_driveSubsystem));

                 final JoystickButton btnTargetOrient = new JoystickButton(controllerPilot,
                                  XboxController.Button.kBack.value);
                 btnTargetOrient.whenPressed(new cmdTargetOrient(m_driveSubsystem), true);
                //  SmartDashboard.putData("Target Orient", new cmdTargetOrient(m_driveSubsystem));
              //   SmartDashboard.putBoolean("Is Target Orient",cmdTargetOrient.isTargetOrient());

                SmartDashboard.putData("Deploy Intake", new cmdIntakeDeploy(m_intakeDeploySubsystem, IntakeConstants.kMaxEncoderPosition));

                // SmartDashboard.putData("Is Robot Square", new cmdIsRobotSquare(m_hopperSubsystem));
                // SmartDashboard.putBoolean("Is Robot Square?", HopperSubsystem.isRobotSquare());  

                // SendableChooser m_isRobotSquare = new SendableChooser<>();

                // m_isRobotSquare.addOption("Yes", new cmdIsRobotSquare(m_hopperSubsystem));
                // m_isRobotSquare.addOption("No", new cmdHopperToggle(m_hopperSubsystem));



        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
        }


        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
        public XboxController getcontrollerPilot() {
                return controllerPilot;
        }

        public XboxController getcontrollerOperator() {
                return controllerOperator;
        }

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // The selected command will be run in autonomous
                return m_chooser.getSelected();
        }

        public Trajectory getTrajectory() {
                if (m_trajectoryChooser.getSelected() != null) {
                        return m_trajectoryChooser.getSelected();
                }
                return m_trajectoryDefault;
        }

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

        public static Alliance getAllianceColor() {
                return allianceColor;
        }
}
