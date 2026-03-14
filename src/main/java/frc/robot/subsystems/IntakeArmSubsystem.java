// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeArmConstants;
import java.lang.Math;

public class IntakeArmSubsystem extends SubsystemBase {

  // Motor and encoder
  // TO DO: replace device ID with constant
  private final SparkMax intakeArmMotor = new SparkMax(IntakeArmConstants.kIntakeArmCanId, MotorType.kBrushless);
  private final RelativeEncoder intakeArmEncoder = intakeArmMotor.getEncoder();
  private final DigitalInput raisedLimitSwitch = new DigitalInput(IntakeArmConstants.kIntakeArmRaisedLimitChannel);

  // PID Controller (tune these constants for your robot)
  // TO DO: replace PID values with constants
  private final PIDController pid = new PIDController(IntakeArmConstants.kPitchP, IntakeArmConstants.kPitchI,
      IntakeArmConstants.kPitchD);
  private static final double kPositionToleranceRot = 0.3; 

  // Conversion factor: encoder rotations → degrees
  // TO DO: setup as constants
  // private static final double GEAR_RATIO = 12.0; // 12:1 gear ratio on arm NEO
  // private static final double DEGREES_PER_REV = 360.0;
  // private static final double POSITION_CONVERSION = DEGREES_PER_REV /
  // GEAR_RATIO;

  // We can change these to what we want after testing
  private static final double STOW_ROT = 0.0;
  private static final double DEPLOY_ROT = 24;

  // Increase this value to allow the motor to deliver a greater output
  private static final double MAX_OUTPUT = 0.9;
  private boolean m_isDeployed = false; // Track whether the arm is currently deployed or stowedgi

  private double output;

  private enum IntakeStates{
    RAISE,
    LOWER,
    STOP}
  private IntakeStates state = IntakeStates.STOP;

  /** Creates a new IntakeArmSubsystem. */
  public IntakeArmSubsystem() {
    // No setPositionConversionFactor available in this REV API; convert rotations
    // to degrees manually
    // Reset encoder to zero at startup (position in rotations
    // Configure PID tolerance
    pid.setTolerance(kPositionToleranceRot);

    m_isDeployed = false;

    // Arm starts in the deployed position, but it needs to be stowed at the
    // beginning of the match, so we set the target rotation to stow
    // stow();
  }
  // TO DO: make private if not used outside this file
  // Raise arm to stow position
  public void stow() {
    // m_isDeployed = false;
    if (!isRaised()) {
      state = IntakeStates.RAISE;
    }
  }

  // TO DO: make private if not used outside this file
  // Lower arm to deploy position
  public void deploy() {
    if (!inTolerance()){
      state = IntakeStates.LOWER;
    }
  }

  public void stopArm() {
    state = IntakeStates.STOP;
    intakeArmMotor.set(0);
  }

  public boolean isRaised() {
    return raisedLimitSwitch.get(); // Assuming the limit switch returns true when pressed
  }

  private boolean inTolerance() {
    double diff = Math.abs(DEPLOY_ROT-intakeArmEncoder.getPosition());
    return (diff < kPositionToleranceRot);
  }


 @Override
  public void periodic() {
    // Reset encoder if the limit switch is pressed
    if (isRaised()) {
      intakeArmEncoder.setPosition(0);
    }

    if (state == IntakeStates.RAISE) {
      if (isRaised()) {
        stopArm();
      } else {
        output = -0.3;
        intakeArmMotor.set(output);
      }
    } else if (state == IntakeStates.LOWER) {
      if (inTolerance()) {
        stopArm();
      } else {
        output = 0.3;
        intakeArmMotor.set(output);
      }
    } else {
      intakeArmMotor.set(0);
    }

    SmartDashboard.putNumber("Intake Encoder", intakeArmEncoder.getPosition());
    SmartDashboard.putString("[INTAKE STATE]", state.name());
    SmartDashboard.putBoolean("Intake Raised Limit", isRaised());
  }
}
