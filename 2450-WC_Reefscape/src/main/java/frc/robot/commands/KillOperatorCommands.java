package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class KillOperatorCommands extends Command {
    CoralSubsystem m_coralSubsystem;
    EndEffectorSubsystem m_endEffectorSubsystem;

    public KillOperatorCommands(CoralSubsystem coralSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        m_coralSubsystem = coralSubsystem;
        m_endEffectorSubsystem = endEffectorSubsystem;

        addRequirements(m_coralSubsystem, m_endEffectorSubsystem);
    }

    public void initialize() {

    }

    public void periodic() {

    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean isFinished) {
        m_coralSubsystem.getElevatorMotorFx().set(0);
        m_endEffectorSubsystem.getEndAffectorMotor().set(0);
    }
}
