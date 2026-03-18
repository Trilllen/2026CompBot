package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

//TO DO: Implement the left auto routine. This will likely involve driving forward, then turning and shooting. You can use the existing DriveSubsystem and ShooterSubsystem commands to help with this.
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftAutoCommand extends Command {
    private final ClimberSubsystem m_robotClimber;
    private final DriveSubsystem m_robotDrive;
    private final IndexerSubsystem m_robotIndexer;
    private final IntakeArmSubsystem m_robotIntakeArm;
    private final LauncherSubsystem m_launcherSubsystem;

  public LeftAutoCommand(ClimberSubsystem climber, DriveSubsystem drive, IndexerSubsystem indexer, IntakeArmSubsystem intakeArm, LauncherSubsystem launcher) {
    m_robotClimber = climber;
    m_robotDrive = drive;
    m_robotIndexer = indexer;
    m_robotIntakeArm = intakeArm;
    m_launcherSubsystem = launcher;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotClimber);
    addRequirements(m_robotDrive);
    addRequirements(m_robotIndexer);
    addRequirements(m_robotIntakeArm);
    addRequirements(m_launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Current autonomous
    m_robotIntakeArm.deploy();
    Commands.waitSeconds(1);
    m_robotClimber.startRetracting();

    // Drive backwards for 2 seconds
    //TO DO: Replace with PathPlanner trajectory following command once trajectories are generated
    //m_robotDrive.drive(-0.3, 0.0, 0.0, true).withTimeout(2.0);

    // Stop drivetrain
    m_robotDrive.drive(0.0, 0.0, 0.0, true);

    // Spin up launcher
    m_launcherSubsystem.startLauncher();
    Commands.waitSeconds(0.6);

    // Feed into shooter
    m_robotIndexer.startIndexerMotor();
    Commands.waitSeconds(5.0);

    // Stop feed and launcher
    m_robotIndexer.stopIndexerMotor();
    m_launcherSubsystem.stopLauncher();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}




