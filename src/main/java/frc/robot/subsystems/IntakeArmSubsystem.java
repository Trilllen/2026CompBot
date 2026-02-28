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
  // TO DO: replace device ID with constant
  private final SparkMax intakeArmMotor = new SparkMax(IntakeArmConstants.kIntakeArmCanId, MotorType.kBrushless);
  private final RelativeEncoder intakeArmEncoder = intakeArmMotor.getEncoder();

  // PID Controller (tune these constants for your robot)
  // TO DO: replace PID values with constants
  private final PIDController pid = new PIDController(IntakeArmConstants.kPitchP, IntakeArmConstants.kPitchI, IntakeArmConstants.kPitchD);
  private static final double kPositionToleranceRot = 0.05; // ±1/20 rotation tolerance

  // Conversion factor: encoder rotations → degrees
  // TO DO: setup as constants
  // private static final double GEAR_RATIO = 12.0; // 12:1 gear ratio on arm NEO
  // private static final double DEGREES_PER_REV = 360.0;
  // private static final double POSITION_CONVERSION = DEGREES_PER_REV /
  // GEAR_RATIO;

  // We can change these to what we want after testing
  private static final double STOW_ROT = 10.0;
  private static final double DEPLOY_ROT = 0.0;

  // Increase this value to allow the motor to deliver a greater output
  private static final double MAX_OUTPUT = 0.9;
  
  // Target rotations
  // TO DO: setup as constant
  private double m_targetRotation;
  private double m_currentRotation;

  private boolean m_isDeployed;
  private boolean m_manualOverride = false;
  private double output;

  /** Creates a new IntakeArmSubsystem. */
  public IntakeArmSubsystem() {
    // No setPositionConversionFactor available in this REV API; convert rotations
    // to degrees manually
    // Reset encoder to zero at startup (position in rotations)
    intakeArmEncoder.setPosition(DEPLOY_ROT);

    // Configure PID tolerance
    pid.setTolerance(kPositionToleranceRot);

    m_targetRotation = STOW_ROT;
    m_currentRotation = DEPLOY_ROT;

    m_isDeployed = true;

    // Arm starts in the deployed position, but it needs to be stowed at the beginning of the match, so we set the target rotation to stow
    stow();
  }

  // TO DO: remove if not used
  public void stop() {
    intakeArmMotor.stopMotor();
  }

  // TO DO: make private if not used outside this file
  public void setTargetRotation(double targetRotation) {
    m_targetRotation = targetRotation;
  }

  // TO DO: make private if not used outside this file
  public void stow() {
    m_isDeployed = false;
    setTargetRotation(STOW_ROT);
  }

  // TO DO: make private if not used outside this file
  public void deploy() {
    m_isDeployed = true;
    setTargetRotation(DEPLOY_ROT);
  }

  public void toggleStowDeploy() {
    if (m_isDeployed) {
      stow();
    } else {
      deploy();
    }
  }

  //May need to flip this signs based off of testing
  public void raiseArm(){
    output = MAX_OUTPUT;
    m_manualOverride = true;
  }
  
  public void lowerArm(){
    output = -MAX_OUTPUT;
    m_manualOverride = true;
  }

  public void stopArm(){
    intakeArmMotor.set(0);
    m_manualOverride = false;
    m_targetRotation = intakeArmEncoder.getPosition();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get motor rotations
    m_currentRotation = intakeArmEncoder.getPosition() * IntakeArmConstants.kPitchEncoderPositionConversionFactor;

    if (!m_manualOverride) {
    output = pid.calculate(m_currentRotation, m_targetRotation);
    }
      
    // Clamp to protect motor
    if (output > MAX_OUTPUT) {
      output = MAX_OUTPUT;
    } else if (output < -MAX_OUTPUT) {
      output = -MAX_OUTPUT;
    }
System.out.println("Current Rot: " + m_currentRotation + " Target Rot: " + m_targetRotation + " Output: " + output);
    intakeArmMotor.set(output);
  }
}
