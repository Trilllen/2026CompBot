package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurretLimeLightCommand extends Command {
    private final TurretSubsystem m_turret;

    public AimTurretLimeLightCommand(TurretSubsystem subsystem) {
        m_turret = subsystem;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        // Get the calculated motor speed from the subsystem logic
        double speed = m_turret.calculateTurretCommand();
        // Apply the speed to the motor
        m_turret.turnTurret(speed);
    }

    @Override
    public boolean isFinished() {
        // Don't finish on our own — let the trigger (holding Y) control lifetime.
        // While the command is scheduled it will continue to call execute() so
        // the turret keeps updating. The Trigger.whileTrue(...) mapping will
        // cancel this command when the Y button is released.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.turnTurret(0); // Stop the turret when command ends
    }
}
