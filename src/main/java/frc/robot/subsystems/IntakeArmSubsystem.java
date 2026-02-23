// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArmSubsystem extends SubsystemBase {

    // Motor and encoder
    //TO DO: replace device ID with constant
    private final SparkMax intakeArmMotor = new SparkMax(IntakeArmConstants.kIntakeArmCanId, MotorType.kBrushless);
    private final RelativeEncoder intakeArmEncoder = intakeArmMotor.getEncoder();

    // PID Controller (tune these constants for your robot)
    //TO DO: replace PID values with constants
    private final PIDController pid = new PIDController(0.05, 0.0, 0.001);
    private static final double kPositionTolerance = 1.0; // degrees

    // Conversion factor: encoder rotations → degrees
    // TO DO: setup as constants
    private static final double GEAR_RATIO = 12.0; // 12:1 gear ratio on arm NEO
    private static final double DEGREES_PER_REV = 360.0;
    private static final double POSITION_CONVERSION = DEGREES_PER_REV / GEAR_RATIO;

    // Target rotations
    // TO DO: setup as constant
    private double targetRotations = 5.0;
    private double currentRotations;

  /** Creates a new ClimberSubsystem. */
  public IntakeArmSubsystem() {
    // No setPositionConversionFactor available in this REV API; convert rotations to degrees manually
    // Reset encoder to zero at startup (position in rotations)
    intakeArmEncoder.setPosition(0);

      // Configure PID tolerance
    pid.setTolerance(kPositionTolerance); // ±1 degree tolerance
  }

  // TO DO: remove if not used
  public void stop() {
      intakeArmMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        currentRotations = intakeArmEncoder.getPosition() * POSITION_CONVERSION;
        intakeArmMotor.set(currentRotations + targetRotations);
  }
}