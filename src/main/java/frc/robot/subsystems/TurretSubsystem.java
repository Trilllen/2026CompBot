// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants; // Assuming you have a Constants file for PID and motor IDs
import frc.robot.utils.LimelightHelpers; // Import the LimelightHelpers class
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.utils.LimelightHelpers.RawFiducial;

public class TurretSubsystem extends SubsystemBase {
    // Establish motor controller object for the turret and encoder
    private final SparkMax m_turretMotor = new SparkMax(TurretConstants.kTurretCanId, SparkMax.MotorType.kBrushless);
    // Spark Max relative encoder attached to the turret motor
    private final RelativeEncoder m_turretEncoder = m_turretMotor.getEncoder();
    // A PID controller for the turret's rotational movement
    private final PIDController m_turretPID = new PIDController(TurretConstants.kP, TurretConstants.kI,
            TurretConstants.kD);
    private LimeLightSubsystem m_limelight; // Reference to the Limelight subsystem to get target data

    public TurretSubsystem(LimeLightSubsystem limelight) {
        // Configure PID controller for continuous input (turret will be limited to 180
        // degrees)
        // m_turretPID.enableContinuousInput(TurretConstants.kMinInput, TurretConstants.kMaxInput); 
        m_limelight = limelight;                                                                                          
        // Set the controller tolerance so we can check when we're "on target"
        m_turretPID.setTolerance(TurretConstants.kTargetToleranceDegrees);
    }

    // Used by both auto aim and Limelight aim
    // output is clamped in the turnTurret method, so we don't set output range here
    public void turnTurret(double speed) {
        // Read current turret angle (degrees) from encoder
        double currentAngle = getTurretAngle();
        SmartDashboard.putNumber("Turret Encoder",currentAngle);

        // System.out.println("[TURRET] Current angle: " + currentAngle + " degrees,
        // Commanded speed: " + speed); // Shows the current angle for the turret

        // Prevent driving past hard limits
        if (speed > 0 && currentAngle >= TurretConstants.kMaxInput) {
            m_turretMotor.set(0);
            return;
        }
        if (speed < 0 && currentAngle <= TurretConstants.kMinInput) {
            m_turretMotor.set(0);
            return;
        }

        double clampSpeed = MathUtil.clamp(speed,
            TurretConstants.kLowClamp,
            TurretConstants.kHighClamp);
            m_turretMotor.set(clampSpeed);
        }

    /**
     * Returns the turret angle in degrees, computed from the Spark Max relative
     * encoder position and the conversion factor in constants. This assumes the
     * encoder position returns rotations.
     */
    public double getTurretAngle() {
        return m_turretEncoder.getPosition() * TurretConstants.kTurretPositionConversion;
    }

    /**
     * Zero the turret encoder at the current position. Useful at startup or
     * when the physical limit switch confirms the mechanical zero position.
     */
    public void zeroTurretEncoder() {
        m_turretEncoder.setPosition(0.0);
    }

    public double getTx() {
        // Get the horizontal offset from the Limelight (returns 0 if no target)

        System.out.println("[TURRET] limelight tx: " + LimelightHelpers.getTX("limelight-tread")); // Debug print to
                                                                                                   // check tx value

        return LimelightHelpers.getTX("limelight-tread"); // Use your Limelight's name if different
    }

    public boolean hasTarget() {
        // Check if the Limelight has a valid target (tv returns 1.0 or 0.0)

        System.out.println("[TURRET] limelight tv: " + LimelightHelpers.getTV("limelight-tread")); // Debug print to
                                                                                                   // check tv value

        return LimelightHelpers.getTV("limelight-tread");
    }

    public void lockOntoHub() {
        // This function is to lock onto the AprilTag on the Hub
        System.out.println("[TURRET] Set to AprilTag on Hub");
        // LimeLightSubsystem.attemptHubLockon();

        // Translation2d m_targetFieldPos =
        // LimeLightSubsystem.getTargetFieldPosition(1); // TO DO: update to use the hub
        // tag ID for the appropriate alliance

        // the commented out code is moved to the LimeLightSubsystem, as it is more
        // appropriate to calculate the target angle there based on the robot's pose and
        // the target's position on the field. The turret subsystem should just receive
        // the desired angle or motor command from the LimeLightSubsystem.
        // // 1. Calculate vector from robot to target
        // Translation2d robotToTarget =
        // m_targetFieldPos.minus(robotPose.getTranslation());

        // // 2. Calculate desired angle relative to field (field-oriented)
        // double desiredAngle = Math.toDegrees(Math.atan2(robotToTarget.getY(),
        // robotToTarget.getX()));

        // // 3. Adjust for current robot heading (gyro) to get absolute field angle
        // double angleToTarget = desiredAngle - robotPose.getRotation().getDegrees();

        // 4. Move turret
        // m_turretMotor.set(m_pid.calculate(m_encoder.getPosition(), angleToTarget));

    }

    public double calculateTurretCommandFromTx(double targetTx) {
        double output = m_turretPID.calculate(targetTx, 0.0);
    
        if (m_turretPID.atSetpoint()) {
            return 0.0;}

        return MathUtil.clamp(output, -1.0, 1.0);
}

    public double calculateTurretCommand(int tagId, double offset) {
            // The 'tx' value is the error (difference from center, in degrees)
            // The PID controller calculates a motor output to make this error zero
            RawFiducial fiducial = m_limelight.getRawFiducialById(tagId);
            double tx = fiducial != null ? fiducial.txnc : 0.0;
            // PIDController.calculate(measurement, setpoint)
            // we want measurement=tx and setpoint=0.0 (center of crosshair)
            double output = m_turretPID.calculate(tx, offset);

            // If we're within the configured tolerance, don't drive the motor (avoid small
            // oscillations)
            if (m_turretPID.atSetpoint()) {
                return 0.0;
            }

            // Optional: Add a feedforward value to overcome static friction
            // output += Constants.Turret.kS;

            // Make sure output is within acceptable limits (e.g., -1.0 to 1.0)
            return MathUtil.clamp(output, -1.0, 1.0);
    }

    @Override
    public void periodic() {
        // This is where you might update Shuffleboard/SmartDashboard with data
    }
}

// public void setAllianceZoneLock() {
// // Code to set turret to alliance zone lock position
// System.out.println("[TURRET] Set to Alliance Zone Lock position");
// }

// public void startStopLauncherMotors(){
// //Starts or stops the launcher motors
// System.out.println("[TURRET] Launcher motors started/stopped");
// }
