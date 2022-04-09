package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class complexAuto extends SequentialCommandGroup {
  /** Creates a new complexDrive. */
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final HopperSubsystem m_hopperSubsystem;
  private final IntakeDeploySubsystem m_intakeDeploySubsystem;
  private double distance;

  public complexAuto(DriveSubsystem driveSubsystem,
                    ShooterSubsystem shooterSubsystem, 
                    HopperSubsystem hopperSubsystem, 
                    IntakeDeploySubsystem IntakeDeploySubsystem, 
                    Double distance) {

    this.m_driveSubsystem = driveSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_hopperSubsystem = hopperSubsystem;
    this.m_intakeDeploySubsystem = IntakeDeploySubsystem;
    this.distance = m_driveSubsystem.getAverageEncoderDistance() + distance;
    addRequirements(m_driveSubsystem);
    addCommands(
      new autoDistanceDrive(m_driveSubsystem, DriveConstants.kAutoDriveDistance),
      new cmdIntakeDeploy(m_intakeDeploySubsystem, IntakeConstants.kMaxEncoderPosition),
      // new cmdShooterSetRPM(ShooterConstants.kShooterPercentOutput,m_shooterSubsystem),
      new cmdShooterToggle(ShooterConstants.kMidOutput,m_shooterSubsystem),
      new cmdTimer(ShooterConstants.kSpinupDelay),
      new cmdHopperToggle(m_hopperSubsystem)
    );
  }
}
