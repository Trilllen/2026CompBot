// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// imports for Rev Robotics MaxSwerve
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.dPadConstants;
import frc.robot.commands.TurretCommands.AimTurretLimeLightCommand;
import frc.robot.commands.TurretCommands.AimTurretManualCommand;
import frc.robot.commands.SnapToAngle;
import frc.robot.commands.TurretCommands.SingleTagAim;
import frc.robot.commands.AutonomousCommands.LeftAutoCommand;
import frc.robot.commands.AutonomousCommands.RightAutoCommand;
import frc.robot.commands.AutonomousCommands.CenterAutoCommand;
import frc.robot.Constants.States;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LauncherHoodSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
// other imports
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

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
        private final IndexerSubsystem m_robotIndexer;
        private final LauncherSubsystem m_launcherSubsystem;
        private final LauncherHoodSubsystem m_launcherHoodSubsystem;

                // Automous options
        private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
        private final Command m_leftAuto;
        private final Command m_rightAuto;
        private final Command m_centerAuto;
        private final Command m_noneAuto = null; // Option for no autonomous routine

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
                m_launcherHoodSubsystem = new LauncherHoodSubsystem();

                // Automous options
                m_leftAuto = new LeftAutoCommand(m_robotClimber, m_robotDrive, m_robotIndexer, m_robotIntakeArm, m_launcherSubsystem);
                m_rightAuto = new RightAutoCommand(m_robotClimber, m_robotDrive, m_robotIndexer, m_robotIntakeArm, m_launcherSubsystem);
                m_centerAuto = new CenterAutoCommand(m_robotClimber, m_robotDrive, m_robotIndexer, m_robotIntakeArm, m_launcherSubsystem);

                setUpAutoCommands();
                
                configureAutoChooser();

                configureButtonBindings();

                // Hood retracts whenever no aiming command is active
                m_launcherHoodSubsystem.setDefaultCommand(
                                new InstantCommand(
                                                () -> m_launcherHoodSubsystem.startRetracting(),
                                                m_launcherHoodSubsystem));

                m_Pigeon.setYaw(0);

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

        // Configure the autonomous command chooser and publish it to the SmartDashboard
        private void configureAutoChooser() {
                // Add options to the chooser
                m_autoChooser.setDefaultOption("Left", m_leftAuto);
                // m_autoChooser.addOption("Center", m_centerAuto);
                // m_autoChooser.addOption("Right", m_rightAuto);
                m_autoChooser.addOption("None", m_noneAuto);

                // Publish the chooser to NetworkTables (SmartDashboard)
                SmartDashboard.putData("Auto Choices", m_autoChooser);
        }

        private void setUpDashboard(){
                SmartDashboard.putData("Flip Heading 180°",
                    new InstantCommand(() -> m_robotDrive.flipHeading(), m_robotDrive)
                        .ignoringDisable(true));
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

                m_gunnerController.x().onTrue(
                                new InstantCommand(
                                                () -> m_robotIntakeArm.stow(),
                                                m_robotIntakeArm));

                m_gunnerController.x().onFalse(
                                new InstantCommand(
                                                () -> m_robotIntakeArm.stopArm(),
                                                m_robotIntakeArm));

                m_gunnerController.b().onTrue(
                                new InstantCommand(
                                                () -> m_robotIntakeArm.deploy(),
                                                m_robotIntakeArm));

                m_gunnerController.b().onFalse(
                                new InstantCommand(
                                                () -> m_robotIntakeArm.stopArm(),
                                                m_robotIntakeArm));

                m_gunnerController.y()
                                .whileTrue(
                                                new AimTurretLimeLightCommand(m_robotTurret, m_Limelight,
                                                                m_currentState, m_launcherSubsystem, m_launcherHoodSubsystem));
                m_gunnerController.a().whileTrue(
                                new SingleTagAim(m_robotTurret, m_Limelight, m_currentState, m_launcherSubsystem, m_launcherHoodSubsystem));

                m_gunnerController.povLeft()
                                .whileTrue(
                                                new StartEndCommand(
                                                                () -> m_robotIndexer.reverseIndexer(),
                                                                () -> m_robotIndexer.stopIndexerMotor(),
                                                                m_robotIndexer));

                m_gunnerController.leftTrigger(OIConstants.kLeftTriggerThreshold)
                                .whileTrue(
                                                new StartEndCommand(
                                                                () -> m_robotIntake.startIntakeRollers(),
                                                                () -> m_robotIntake.stopIntakeRollers(),
                                                                m_robotIntake));

                m_gunnerController.leftBumper()
                                .whileTrue(new RunCommand(() -> m_robotIntake.reverseIntakeRollers(),
                                        () -> m_robotIntake.stopIntakeRollers(),
                                        m_robotIntake));

                //
                m_robotTurret.setDefaultCommand(new AimTurretManualCommand(
                                m_robotTurret,
                                () -> MathUtil.applyDeadband(m_gunnerController.getLeftX(),
                                                OIConstants.kGunnerDeadBand)));
        }

        private void setUpAutoCommands() {
                NamedCommands.registerCommand("lowerIntake",
                                new InstantCommand(
                                                () -> m_robotIntakeArm.deploy(),
                                                m_robotIntakeArm));

                NamedCommands.registerCommand("startIndexer",
                                new InstantCommand(
                                                () -> m_robotIndexer.startIndexerMotor(),
                                                m_robotIndexer));

                NamedCommands.registerCommand("stopIndexer",
                                new InstantCommand(
                                                () -> m_robotIndexer.stopIndexerMotor(),
                                                m_robotIndexer));

                NamedCommands.registerCommand("startLauncher",
                                new InstantCommand(
                                                () -> m_launcherSubsystem.startLauncher(),
                                                m_launcherSubsystem));

                NamedCommands.registerCommand("stopLauncher",
                                new InstantCommand(
                                                () -> m_launcherSubsystem.stopLauncher(),
                                                m_launcherSubsystem));

                NamedCommands.registerCommand("retractClimber",
                                new InstantCommand(
                                                () -> m_robotClimber.startRetracting(),
                                                m_robotClimber));
                // Set up any autonomous commands or command groups here
        }

        /*
         * Gunner
         * A -> Single tag aim (hold)
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

                return new PathPlannerAuto("leftAuto");
                
        }
}
