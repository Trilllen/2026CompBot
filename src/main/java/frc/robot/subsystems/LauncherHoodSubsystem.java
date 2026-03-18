import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
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
  private double maxSetpoint = LauncherConstants.kHoodMaxSetPoint;
  private final PIDController m_hoodPID = new PIDController(Launcher7Constants.kPHood, TurretConstants.kIHood,
            TurretConstants.kDHood);
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

  public void goToSetpoint(double setpointRaw) {
    setpoint = minSetpoint+(PWMrange*setpointRAW);
    m_hoodPID.setSetpoint(setpoint);
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
      case STOPPED;
        m_hoodcontroller.set(0.0)
    
    
    
    }
    
  }

}
