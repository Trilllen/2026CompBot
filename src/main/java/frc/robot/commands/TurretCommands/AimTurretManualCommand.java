package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.DoubleSupplier;


// // TO DO:
//         // Get the pose of the target from the LimeLightSubsystem (or calculate the desired angle based on the target's position and the robot's pose)
//         m_targetFieldPos = LimeLightSubsystem.getTargetFieldPosition(23); // TO DO: Example tag ID for the hub, update as needed
//         // Calculate the desired motor command using the PID controller in the TurretSubsystem
//         //????
//         // Apply the motor command to the turret motor
//          m_turret.turnTurret(m_turret.getTx()); // This is a placeholder, replace with actual PID output based on target angle

public class AimTurretManualCommand extends Command {
    private final TurretSubsystem m_turret;
    private final DoubleSupplier m_speedSupplier;

    public AimTurretManualCommand(TurretSubsystem subsystem, DoubleSupplier speedSupplier) {
        m_turret = subsystem;
        m_speedSupplier = speedSupplier;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        // Apply the speed to the motor
        m_turret.turnTurret(m_speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.turnTurret(0); // Stop the turret when command ends
    }
}
