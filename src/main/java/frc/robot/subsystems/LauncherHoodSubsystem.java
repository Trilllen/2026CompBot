package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.LauncherConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class LauncherHoodSubsystem extends SubsystemBase {
  enum States {
    STOPPED,
    FULLRETRACT,
    FULLEXTEND,
    GOTOSETPOINT
  }
  private States state = States.STOPPED;
  private int canID = LauncherConstants.kHoodControllerCanId;
  private SparkMax m_hoodController = new SparkMax(canID, MotorType.kBrushed);
  private AnalogInput reading = new AnalogInput(LauncherConstants.kHoodPort);
  private double minSetpoint = LauncherConstants.kHoodMinSetpoint;
  private double maxSetpoint = LauncherConstants.kHoodMaxSetpoint;
  private final PIDController m_hoodPID = new PIDController(LauncherConstants.kPHood, LauncherConstants.kIHood,
            LauncherConstants.kDHood);
  private double PWMrange = Math.abs(maxSetpoint-minSetpoint);
  
  public LauncherHoodSubsystem() {
    m_hoodPID.setSetpoint(minSetpoint);
  }
  
  public void startRetracting() {
    if(!atMin()){
      state = States.FULLRETRACT;
      m_hoodPID.setSetpoint(minSetpoint);
    }    
  }

  public void startExtending() {
    if (!atMax()){
      state = States.FULLEXTEND;
      m_hoodPID.setSetpoint(maxSetpoint);
    }
  }


  /**
 * @param desiredExtension normalized [0–1], not voltage
 */
  public void goToSetpoint(double desiredExtension) {
    double targetSetpoint = minSetpoint+(PWMrange*desiredExtension);
    m_hoodPID.setSetpoint(targetSetpoint);
    state = States.GOTOSETPOINT;
  }

  public void stop() {
    state = States.STOPPED;
    m_hoodController.set(0.0);
  }

  public double getPosition() {
    return reading.getVoltage();
  }

  public boolean atMin() {
    return getPosition() <= minSetpoint;
  }

  public boolean atMax() {
    return getPosition() >= maxSetpoint;
  }

  @Override
  public void periodic() {
    double currentPosition = getPosition();
    double output = 0;
    switch (state) {
      case STOPPED:
        m_hoodController.set(0.0);
        m_hoodPID.setSetpoint(getPosition());
        break;
        
      case FULLRETRACT:
        if (!atMin()){
        } else {
          stop();
        }
          break;
        
      case FULLEXTEND:
        if (!atMax()){
        } else {
          stop();
        }
        break;
        
      case GOTOSETPOINT:
        if (m_hoodPID.atSetpoint()){
          stop();}
        break;
        
      }
    output = m_hoodPID.calculate(currentPosition);
    m_hoodController.set(output);

    SmartDashboard.putNumber("Hood Position", currentPosition);
    SmartDashboard.putNumber("Hood Setpoint", m_hoodPID.getSetpoint());
    SmartDashboard.putString("Hood State", state.name());
    
  }

}
