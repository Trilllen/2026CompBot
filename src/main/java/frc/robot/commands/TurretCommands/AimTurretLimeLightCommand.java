package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.ZoneData;
import frc.robot.subsystems.LimeLightSubsystem.HubZone;
import frc.robot.utils.LimelightHelpers.RawFiducial;

import java.util.ArrayList;

public class AimTurretLimeLightCommand extends Command {
    private TurretSubsystem m_turret;
    private LimeLightSubsystem m_limelight;
    private boolean isRed;

    public AimTurretLimeLightCommand(TurretSubsystem turret, LimeLightSubsystem limelight) {
        m_turret = turret;
        m_limelight = limelight;
        isRed = m_limelight.isRedAlliance();
        addRequirements(m_turret);
    }

    public double calculateTurretCommand() { // Get the current angle to the hub from the Limelight subsystem
        double tx = 0.0;
        return tx;
    }

    public boolean checkForTags(int tag1, int tag2) {
        ArrayList<Integer> activeTags = m_limelight.getVisibleAprilTagIDs();
        return activeTags.contains(tag1) && activeTags.contains(tag2);
    }

    public double interpolate(ZoneData data){
        double averageAngle = (data.leftAngle() + data.rightAngle()) / 2.0;
        double distance = m_limelight.getDistanceToHub();
        double currentAngle = m_limelight.getAngleToHub();
        double angleInterp = (currentAngle - data.leftAngle()) / (data.rightAngle() - data.leftAngle());
        return angleInterp;
    }
    public double getTagSpacing(ZoneData data){
        int leftTag = data.leftTag();
        int rightTag = data.rightTag();
        RawFiducial leftTagData = m_limelight.getRawFiducialById(leftTag);
        double leftTagTXNC = 0.0;
        if (leftTagData != null){
            leftTagTXNC = leftTagData.txnc;
        }
        RawFiducial rightTagData = m_limelight.getRawFiducialById(rightTag);
        double rightTagTXNC = 0.0;
        if (rightTagData != null){
            rightTagTXNC = rightTagData.txnc;
        }
        double spacing = leftTagTXNC - rightTagTXNC;
        return spacing;
        
    }

    @Override
    public void execute() {
        // Get the calculated motor speed from the subsystem logic
        // ZoneData contains (HubZone zone, double leftAngle, int leftTag, double
        // rightAngle, int rightTag)
        ZoneData data = m_limelight.getHubZoneData();
        // Check if both tags are being tracked
        boolean bothTagsSeen = checkForTags(data.leftTag(), data.rightTag());
        if (bothTagsSeen) {
            // If both tags are seen, we can use the average of the left and right angles for better accuracy
            double interpolation = interpolate(data);
            double tagSpacing = getTagSpacing(data);
            double offset = tagSpacing * interpolation;

            double speed = m_turret.calculateTurretCommand(0);
            // Apply the speed to the motor
            m_turret.turnTurret(speed);
        }
    }

    @Override
    public boolean isFinished() {
        // Don't finish on our own — let the trigger (holding Y) control lifetime.
        // While the command is scheduled it will continue to call execute() so
        // the turret keeps updating. The Trigger.whileTrue(...) mapping will
        // cancel this command when the Y button is released. return false;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.turnTurret(0); // Stop the turret when command ends
    }
}
