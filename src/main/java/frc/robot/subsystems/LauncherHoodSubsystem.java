package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.ShootingConstants;
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
    // Initialize dashboard entries so Elastic can display them as widgets
    SmartDashboard.putBoolean("Hood Manual Mode", false);
    SmartDashboard.putNumber("Hood Manual Setpoint", 0.0);
    // Open-loop debug controls to help diagnose motor/wiring/sign issues
    SmartDashboard.putBoolean("Hood Open Loop Enabled", false);
    SmartDashboard.putNumber("Hood Open Loop Percent", 0.0);
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

  /** Looks up the correct hood angle for the given distance and moves to it. */
  public void setHoodForDistance(double distanceMeters) {
    double targetAngle = ShootingConstants.interpolate(
        ShootingConstants.kDistanceMeters,
        ShootingConstants.kHoodAngles,
        distanceMeters);
    goToSetpoint(targetAngle);
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

    // --- Testing only: manual hood control from SmartDashboard/Elastic ---
    // Toggle "Hood Manual Mode" and adjust "Hood Manual Setpoint" [0-1] in Elastic.
    boolean manualMode = SmartDashboard.getBoolean("Hood Manual Mode", false);
    double manualSetpoint = SmartDashboard.getNumber("Hood Manual Setpoint", 0.0);

    if (manualMode) {
      m_hoodPID.setSetpoint(minSetpoint + (PWMrange * manualSetpoint));
      double pidOut = m_hoodPID.calculate(currentPosition);
      m_hoodController.set(-pidOut);
      SmartDashboard.putNumber("hoodPID", pidOut);
      SmartDashboard.putNumber("Hood Position", currentPosition);
      SmartDashboard.putNumber("Hood Setpoint", manualSetpoint);
      SmartDashboard.putString("Hood State", "MANUAL");
      SmartDashboard.putNumber("hood motor controller output", m_hoodController.get());
      return; // skip normal state machine while in manual mode
    }

    // --- Open-loop diagnostic mode (bypasses PID) ---
    boolean openLoopEnabled = SmartDashboard.getBoolean("Hood Open Loop Enabled", false);
    double openLoopPercent = SmartDashboard.getNumber("Hood Open Loop Percent", 0.0);
    if (openLoopEnabled) {
      // clamp to [-1,1]
      double clamped = Math.max(-1.0, Math.min(1.0, openLoopPercent));
      m_hoodController.set(clamped);
      SmartDashboard.putNumber("Hood Position", currentPosition);
      SmartDashboard.putNumber("Hood Setpoint", m_hoodPID.getSetpoint());
      SmartDashboard.putString("Hood State", "OPEN_LOOP");
      SmartDashboard.putNumber("hood motor controller output", m_hoodController.get());
      return;
    }

    double output = 0;

    // State machine for hood control
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
  double pidOutput = m_hoodPID.calculate(currentPosition);
  output = -pidOutput;
  m_hoodController.set(output);

    SmartDashboard.putNumber("Hood Position", currentPosition);
    SmartDashboard.putNumber("Hood Setpoint", m_hoodPID.getSetpoint());
    SmartDashboard.putString("Hood State", state.name());
    
  }

}
