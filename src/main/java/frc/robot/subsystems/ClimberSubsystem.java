// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  SparkMax climberMotor = new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
  RelativeEncoder climberEncoder = climberMotor.getEncoder();
  // Limit switch on the retract (bottom) side of the climber. When this
  // DigitalInput returns true (or false depending on wiring) the climber
  // is fully retracted and we should not drive further inward.
  private final DigitalInput m_retractLimit = new DigitalInput(ClimberConstants.kClimberRetractLimitChannel);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  }

  // TO DO: add code to limit the extension and retraction of the climber, either
  // through encoder values or limit switches, to prevent damage to the mechanism.
  // For now, we will just have the motors run at the given speed when extending
  // or retracting, and stop when the stopClimber function is called.
  public void extendClimber(double speed) {
    // Code to extend the climber mechanism
    // Allow extension regardless of the retract limit switch state
    // (retract limit only prevents further retraction)
    // System.out.println("[CLIMBER] Extending climber");
    climberMotor.set(speed);
    // may need this to prevent overextension
    // if (getHeightInches() > ElevatorConstants.kMaxPos) {
    // stopMotors();
    // }
    System.out.println("[CLIMBER] Climber height (Climber posistion): " + climberEncoder.getPosition());
  }

  // TO DO: add code to limit the extension and retraction of the climber, either
  // through encoder values or limit switches, to prevent damage to the mechanism.
  // For now, we will just have the motors run at the given speed when extending
  // or retracting, and stop when the stopClimber function is called.
  public void retractClimber(double speed) {
    // Code to retract the climber mechanism
    // If the retract limit is hit we must not attempt to retract further.
    if (isRetracted()) {
      // Already retracted; ensure motor is stopped
      stopClimber();
      climberEncoder.setPosition(0); // Reset encoder position to 0 at the retracted limit
      return;
    }

    // System.out.println("[CLIMBER] Retracting climber");
    climberMotor.set(-speed);

  }

  public void stopClimber() {
    // Code to stop the climber mechanism // Set Speed to 0
    // System.out.println("[CLIMBER] Stopping climber");
    climberMotor.set(0);
  }

  /**
   * Returns true when the retract limit switch is triggered. Depending on
   * wiring the DigitalInput may return true when pressed or when released.
   * If your switch is wired NC/NO differently, invert this logic here.
   */
  public boolean isRetracted() {
    System.out.println("Limit switch = " + m_retractLimit.get());
    // Return true when the physical retract switch is pressed. The wiring
    // (NO vs NC) can change whether get() returns true or false when pressed,
    // so compare against the team-configurable constant in Constants.
    return m_retractLimit.get() == ClimberConstants.kRetractLimitPressedHigh;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}