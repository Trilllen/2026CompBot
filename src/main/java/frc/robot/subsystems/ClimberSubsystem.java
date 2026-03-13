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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax climberMotor = new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder climberEncoder = climberMotor.getEncoder();
  // Limit switch on the retract (bottom) side of the climber. When this
  // DigitalInput returns true (or false depending on wiring) the climber
  // is fully retracted and we should not drive further inward.
  private final DigitalInput m_retractLimit = new DigitalInput(ClimberConstants.kClimberRetractLimitChannel);
  
  private enum ClimberStates {
    EXTENDING,
    RETRACTING,
    STOPPED
  }
  private ClimberStates state = ClimberStates.STOPPED;
  private double retractionSpeed = ClimberConstants.kClimberReverseMotorSpeed;
  private double extensionSpeed = ClimberConstants.kClimberMotorSpeed;
  private double climberMaxHeight = ClimberConstants.kClimberEncoderHighPoint;
  

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  }

  // TO DO: add code to limit the extension and retraction of the climber, either
  // through encoder values or limit switches, to prevent damage to the mechanism.
  // For now, we will just have the motors run at the given speed when extending
  // or retracting, and stop when the stopClimber function is called.
  private void extendClimber() {
    // Code to extend the climber mechanism
    // Allow extension regardless of the retract limit switch state
    climberMotor.set(extensionSpeed);
  }
  
  private void retractClimber() {
    climberMotor.set(retractionSpeed);
  }
  
  public void startRetracting() {
    if (!isRetracted())
      state = ClimberStates.RETRACTING;
  }
  
  public void startExtending() {
      if (climberEncoder.getPosition() < climberMaxHeight){
        state = ClimberStates.EXTENDING;        
      }
  }

  public void stopClimber() {
    // Code to stop the climber mechanism
    climberMotor.set(0);
    state = ClimberStates.STOPPED;
  }

  /**
   * Returns true when the retract limit switch is triggered. Depending on
   * wiring the DigitalInput may return true when pressed or when released.
   * If your switch is wired NC/NO differently, invert this logic here.
   */
  public boolean isRetracted() {
    
    // Return true when the physical retract switch is pressed. The wiring
    // so compare against the team-configurable constant in Constants.
    return m_retractLimit.get() == ClimberConstants.kRetractLimitPressedHigh;
  }

  @Override
  public void periodic() {
    //Retraction logic    
    if (state == ClimberStates.RETRACTING) {
      // stop if we've hit the limit
      if(isRetracted()) {
        stopClimber();
        //if limit isn't hit, retract
      } else {
        retractClimber();
      }
    }
    //Extension logic
    if (state == ClimberStates.EXTENDING){
      if (climberEncoder.getPosition() >= climberMaxHeight) {
        stopClimber();
      } else {
        extendClimber();
      }
    }
    SmartDashboard.putBoolean("Climber Limit switch", m_retractLimit.get());
    SmartDashboard.putNumber("[CLIMBER ENCODER]", climberEncoder.getPosition());
    SmartDashboard.putString("[CLIMBER STATE]", state.name());
  }
}
