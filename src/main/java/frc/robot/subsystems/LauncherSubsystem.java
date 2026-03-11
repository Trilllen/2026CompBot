// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.States;
import frc.robot.Constants.States.State;



public class LauncherSubsystem extends SubsystemBase {
  /** Creates a new LauncherSubsystem. */

  // Declare the TalonFX motor controller object with a specific CAN ID
  private final TalonFX m_krakenMotorMaster = new TalonFX(LauncherConstants.kLauncherMotorMaster); // Replace 10 with your motor's CAN ID
  private final TalonFX m_krakenMotorFollower = new  TalonFX (LauncherConstants.kLauncherMotorFollower);
  public States m_currentState;
  private boolean throttle;
  private boolean launcherIsOn = false;

  public LauncherSubsystem(States state) {

    m_currentState = state;
    m_krakenMotorFollower.setControl(new Follower(m_krakenMotorMaster.getDeviceID(), MotorAlignmentValue.Opposed));
    throttle = LauncherConstants.kLauncherMotorSpeed;
    SmartDashboard.putNumber("[THROTTLE]", throttle);

    // we may not need to use the Config lines
    // Optional: Configure motor inversion if needed
    MotorOutputConfigs motorOutputMaster = new MotorOutputConfigs();
    motorOutputMaster.Inverted = InvertedValue.Clockwise_Positive; // or
    // CounterClockwise_Positive
    m_krakenMotorMaster.getConfigurator().apply(motorOutputMaster);

    MotorOutputConfigs motorOutputFollower = new MotorOutputConfigs();
    motorOutputFollower.Inverted = InvertedValue.Clockwise_Positive;
    m_krakenMotorFollower.getConfigurator().apply(motorOutputFollower);
  }

  public void startLauncher() {
    // Starts the motor to the set value
    m_currentState.setState(State.Launching);
    launcherIsOn = true;
  }

  public void stopLauncher() {
    // Stops the motor
    m_currentState.setState(State.Initial);
    m_krakenMotorMaster.set(0);
    launcherIsOn = false;
  }

  public void reverseLauncher() {
    throttle = -LauncherConstants.kLauncherReverseMotorSpeed;
    SmartDashboard.putNumber("[THROTTLE]", throttle);
    launcherIsOn = true;
  }

  @Override
  public void periodic() {

    if (launcherIsOn){
      throttle SmartDashboard.getNumber("[THROTTLE]", 0.1);
      m_krakenMotorMaster.set(throttle);
    }else{
      m_krakenMotorMaster.set(0);
    }
  }
}
