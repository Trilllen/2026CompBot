import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SnapToAngle extends Command {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final DoubleSupplier m_rotSupplier;
    private final Rotation2d m_targetHeading;
      
    public SnapToAngle(
        DriveSubsystem drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier rotSupplier,
        Rotation2d targetHeading) {
      m_drive = drive;
      m_xSupplier = xSupplier;
      m_ySupplier = ySupplier;
      m_rotSupplier = rotSupplier;
      m_targetHeading = targetHeading;
  
      addRequirements(m_drive);
    }

    @Override
    public void initialize() {
      m_drive.resetHeadingController();
    }

    @Override
    public void execute() {
      double x = MathUtil.applyDeadband(m_xSupplier.getAsDouble(), OIConstants.kDriveDeadband);
      double y = MathUtil.applyDeadband(m_ySupplier.getAsDouble(), OIConstants.kDriveDeadband);
  
      double rot = m_drive.calculateTurnToHeading(m_targetHeading.getDegrees());
  
      m_drive.drive(x, y, rot, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
    double manualRot = MathUtil.applyDeadband(m_rotSupplier.getAsDouble(), OIConstants.kDriveDeadband);

    // Finish if driver starts rotating manually,
    // or if we've reached the target heading.
    return Math.abs(manualRot) > 0.05 || m_drive.atHeadingSetpoint();
  }
}
