// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// imports for Rev Robotics MaxSwerve
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.dPadConstants;
import frc.robot.commands.TurretCommands.AimTurretLimeLightCommand;
import frc.robot.commands.TurretCommands.AimTurretManualCommand;
import frc.robot.commands.SnapToAngle;
import frc.robot.Constants.States;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.UpperIndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// other imports
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

        public CANBus m_CanBus = new CANBus();
        public Pigeon2 m_Pigeon;
        public States m_currentState;

        // The robot's subsystems and commands are defined here...

        // MOVED TO INSIDE public RobotContainer(), because need to send pigeon as
        // parameter.
        private final DriveSubsystem m_robotDrive;
        private final IntakeSubsystem m_robotIntake;
        private final IntakeArmSubsystem m_robotIntakeArm;
        private final TurretSubsystem m_robotTurret;
        private final ClimberSubsystem m_robotClimber;
        private final LimeLightSubsystem m_Limelight;
        private final LEDSubsystem m_LedSubsystem;
        private final IndexerSubsystem m_robotIndexer;
        private final LauncherSubsystem m_launcherSubsystem;
        private final UpperIndexerSubsystem m_UpperIndexerSubsystem;

        // The driver's controller
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);
        private final CommandXboxController m_gunnerController = new CommandXboxController(
                        OIConstants.kGunnerControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Initialize subsystems
                m_Pigeon = new Pigeon2(Constants.SensorConstants.kPigeonCanId, m_CanBus);
                m_currentState = new States();
                DriverStation.silenceJoystickConnectionWarning(true);

                m_robotDrive = new DriveSubsystem(m_Pigeon);
                m_robotIntake = new IntakeSubsystem();
                m_robotIntakeArm = new IntakeArmSubsystem();
                m_Limelight = new LimeLightSubsystem(m_robotDrive);
                m_robotTurret = new TurretSubsystem(m_Limelight);
                m_robotClimber = new ClimberSubsystem();
                m_launcherSubsystem = new LauncherSubsystem(m_currentState);
                m_robotIndexer = new IndexerSubsystem();
                m_UpperIndexerSubsystem = new UpperIndexerSubsystem();
                m_LedSubsystem = new LEDSubsystem(m_currentState);
                configureButtonBindings();

                // Configure default commands
                if (m_robotDrive != null) {
                        m_robotDrive.setDefaultCommand(
                                        // The left stick controls translation of the robot.
                                        // Turning is controlled by the X axis of the right stick
                                        new RunCommand(
                                                        () -> {
                                                                double slow = m_driverController.leftBumper()
                                                                                .getAsBoolean() ? 0.2 : 1.0;

                                                                double xSpeed = -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband) * slow;
                                                                double ySpeed = -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband) * slow;
                                                                double rot = -MathUtil.applyDeadband(
                                                                                m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband) * slow;

                                                                m_robotDrive.drive(xSpeed, ySpeed, rot, true);
                                                        },
                                                        m_robotDrive));
                }
        }

        /**
         * Use this method to define your button->command mappings using
         * CommandXboxController's built-in Trigger methods.
         */
        private void configureButtonBindings() {
                /*
                 * DRIVER CONTROLS
                 */

                // Right bumper -> lock wheels in X pattern
                m_driverController.rightBumper()
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));

                if (Constants.kTestMode) {
                        // Start button -> zero heading
                        m_driverController.start()
                                        .onTrue(new InstantCommand(
                                                        () -> m_robotDrive.zeroHeading(),
                                                        m_robotDrive));
                }

                m_driverController.povUp().onTrue(
                                new SnapToAngle(
                                                m_robotDrive,
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX(),
                                                () -> -m_driverController.getRightX(),
                                                Rotation2d.fromDegrees(0)));

                m_driverController.povRight().onTrue(
                                new SnapToAngle(
                                                m_robotDrive,
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX(),
                                                () -> -m_driverController.getRightX(),
                                                Rotation2d.fromDegrees(-90)));

                m_driverController.povDown().onTrue(
                                new SnapToAngle(
                                                m_robotDrive,
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX(),
                                                () -> -m_driverController.getRightX(),
                                                Rotation2d.fromDegrees(180)));

                m_driverController.povLeft().onTrue(
                                new SnapToAngle(
                                                m_robotDrive,
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX(),
                                                () -> -m_driverController.getRightX(),
                                                Rotation2d.fromDegrees(90)));

                /*
                 * GUNNER CONTROLS
                 */

                // Right bumper (analog) -> run launcher (hold)
                m_gunnerController.rightBumper()
                                .whileTrue(new StartEndCommand(
                                                () -> m_launcherSubsystem.startLauncher(),
                                                () -> m_launcherSubsystem.stopLauncher(),
                                                m_launcherSubsystem));

                // Right bumper -> run indexer (hold)
                // Upper indexer is set to follow lower indexer on the Rev Hardware Client
                m_gunnerController.rightTrigger(OIConstants.kRightTriggerThreshold)
                                .whileTrue(new StartEndCommand(
                                                () -> m_robotIndexer.startIndexerMotor(),
                                                () -> m_robotIndexer.stopIndexerMotor(),
                                                m_robotIndexer));

                // D-Pad Up -> extend climber
                m_gunnerController.povUp().onTrue(
                        new InstantCommand(
                                () -> m_robotClimber.startExtending(),
                                m_robotClimber));
                
                m_gunnerController.povUp().onFalse(
                        new InstantCommand(
                                () -> m_robotClimber.stopClimber(),
                                m_robotClimber));
                // D-Pad Down -> retract climber
                m_gunnerController.povDown().onTrue(
                        new InstantCommand(
                                () -> m_robotClimber.startRetracting(),
                                m_robotClimber));

                m_gunnerController.povDown().onFalse(
                        new InstantCommand(
                                () -> m_robotClimber.stopClimber(),
                                m_robotClimber));

                // A button -> toggle intake arm stow/deploy
                m_gunnerController.a()
                                .onTrue(new InstantCommand(() -> m_robotIntakeArm.toggleStowDeploy(),
                                                m_robotIntakeArm));

                // MANUAL arm
                m_gunnerController.x()
                                .whileTrue(
                                                new StartEndCommand(
                                                                () -> m_robotIntakeArm.raiseArm(),
                                                                () -> m_robotIntakeArm.stopArm(),
                                                                m_robotIntakeArm));
                m_gunnerController.b()
                                .whileTrue(
                                                new StartEndCommand(
                                                                () -> m_robotIntakeArm.lowerArm(),
                                                                () -> m_robotIntakeArm.stopArm(),
                                                                m_robotIntakeArm));
                m_gunnerController.y()
                                .whileTrue(
                                                new AimTurretLimeLightCommand(m_robotTurret, m_Limelight,
                                                                m_currentState));
                // D-Pad Left -> reverse indexer (while held)
                m_gunnerController.pov(dPadConstants.kDPadLeft)
                                .whileTrue(
                                                new StartEndCommand(
                                                                () -> m_robotIndexer.reverseIndexer(),
                                                                () -> m_robotIndexer.stopIndexerMotor(),
                                                                m_robotIndexer));

                // D-Pad Right -> reverse launcher (while held)
                m_gunnerController.pov(dPadConstants.kDPadRight)
                                .whileTrue(
                                                new StartEndCommand(
                                                                () -> m_launcherSubsystem.reverseLauncher(),
                                                                () -> m_launcherSubsystem.stopLauncher(),
                                                                m_launcherSubsystem));

                // Left trigger -> start/stop intake rollers
                m_gunnerController.leftTrigger(OIConstants.kLeftTriggerThreshold)
                                .toggleOnTrue(
                                                new StartEndCommand(
                                                                () -> m_robotIntake.startIntakeRollers(),
                                                                () -> m_robotIntake.stopIntakeRollers(),
                                                                m_robotIntake));
                // changed to toggle, see code above
                // m_gunnerController.leftTrigger(OIConstants.kLeftTriggerThreshold)
                // .onTrue(new InstantCommand(() -> m_robotIntake.startIntakeRollers(),
                // m_robotIntake));
                // m_gunnerController.leftTrigger(OIConstants.kLeftTriggerThreshold)
                // .onFalse(new InstantCommand(() -> m_robotIntake.stopIntakeRollers(),
                // m_robotIntake));

                // Left bumper -> reverse intake rollers (while held)
                m_gunnerController.leftBumper()
                                .whileTrue(new RunCommand(() -> m_robotIntake.reverseIntakeRollers(), m_robotIntake));

                //
                m_robotTurret.setDefaultCommand(new AimTurretManualCommand(
                                m_robotTurret,
                                () -> MathUtil.applyDeadband(m_gunnerController.getLeftX(),
                                                OIConstants.kGunnerDeadBand)));
        }

        /*
         * Gunner
         * A -> Extend/Retract Intake (toggle)
         * B -> Arm down (hold)
         * X -> Arm manual up (hold)
         * Y -> Lock onto hub (hold) TO DO
         * LB -> Reverse Intake Rollers (hold)
         * RB -> Launcher on and off (hold)
         * upper and lower indexer motor, instead of just while held
         * LeftJoystick ->
         * LeftJoystickClick ->
         * RightJoystick -> Aim launcher
         * RightJoystickClick ->
         * DPad Up -> Climber extend (hold)
         * DPad Down -> Climber retract (hold)
         * DPad Left -> Reverse Indexer (hold)
         * DPad Right -> Reverse launcher (hold)
         * LeftTrigger -> Start/stop Intake Rollers (toggle)
         * RightTrigger -> Toggle indexers on and off (TOGGLE)
         * launcher motors, instead of just while held
         * StartButton -> Start/Stop launcher motors (toggle)
         * 
         */
        /*
         * Driver
         * A ->
         * B ->
         * X ->
         * Y -> a
         * LB ->
         * RB ->
         * LeftJoystick -> Motion of robot
         * RightJoystick -> Rotation of robot
         * DPad Up ->
         * DPad Down ->
         * DPad Left ->
         * DPad Right ->
         * LeftTrigger -> Brake/Slow down (go slower when held more)? Dont really need
         * but maybe
         * RightTrigger ->
         * 
         */

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                // /**
                // * Use this to pass the autonomous command to the main {@link Robot} class.
                // *
                // * @return the command to run in autonomous
                // */
                // return autoChooser.getSelected();

                /*
                 * Autonomouse to start launcher, move forward 2 feet, then run upper indexer,
                 * then run indexer, wait 4 seconds, then stop motors.
                 */
                Command m_autonomousCommand;
                m_autonomousCommand =
                                // new PathPlannerAuto("Dead Ahead")
                                // .andThen(() -> m_ElevatorSubsystem.goToElevatorL2(), m_ElevatorSubsystem)
                                new InstantCommand(() -> m_launcherSubsystem.startLauncher(), m_launcherSubsystem)
                                                .andThen(Commands.waitSeconds(2))
                                                // .andthen(m_robotDrive.)
                                                // .andThen(() -> m_UpperIndexerSubsystem.startUpperIndexerMotor(),
                                                // m_UpperIndexerSubsystem)
                                                .andThen(() -> m_robotIndexer.startIndexerMotor(), m_robotIndexer)
                                                .andThen(Commands.waitSeconds(5))
                                                .andThen(() -> m_robotIndexer.stopIndexerMotor(), m_robotIndexer)
                                                // .andThen(() -> m_UpperIndexerSubsystem.stopUpperIndexerMotor(),
                                                // m_UpperIndexerSubsystem)
                                                .andThen(() -> m_launcherSubsystem.stopLauncher(), m_launcherSubsystem);
                return m_autonomousCommand;

                // this is code from 2025. is it needed/useful?
                // // Create config for trajectory
                // TrajectoryConfig config = new TrajectoryConfig(
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(DriveConstants.kDriveKinematics);

                // // An example trajectory to follow. All units in meters.
                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(3, 0, new Rotation2d(0)),
                // config);

                // var thetaController = new ProfiledPIDController(
                // AutoConstants.kPThetaController, 0, 0,
                // AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // m_robotDrive::getPose, // Functional interface to feed supplier
                // DriveConstants.kDriveKinematics,

                // // Position controllers
                // new PIDController(AutoConstants.kPXController, 0, 0),
                // new PIDController(AutoConstants.kPYController, 0, 0),
                // thetaController,
                // m_robotDrive::setModuleStates,
                // m_robotDrive);

                // // Reset odometry to the starting pose of the trajectory.
                // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // // Run path following command, then stop at the end.
                // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
                // false));
        }

}
