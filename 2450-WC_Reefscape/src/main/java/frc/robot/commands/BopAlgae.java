package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class BopAlgae extends Command {
    EndEffectorSubsystem m_EndEffectorSubsystem;
    DoubleSupplier m_forwardSpeed;
    DoubleSupplier m_backwardSpeed;

    String m_direction;

    public BopAlgae(EndEffectorSubsystem endEffectorSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier backwardSpeed) {
        m_EndEffectorSubsystem = endEffectorSubsystem;
        m_backwardSpeed = backwardSpeed;
        m_forwardSpeed = forwardSpeed;

        addRequirements(m_EndEffectorSubsystem);
    }

    public void initialize() {
    }

    public void execute() {
        double fS = (m_backwardSpeed.getAsDouble() - m_forwardSpeed.getAsDouble()) * 0.5;
        m_EndEffectorSubsystem.setEndAffectorSpeed(fS);
    }

    public void end(boolean interrupted) {
        m_EndEffectorSubsystem.setEndAffectorSpeed(0);
    }

    public boolean isFinished() {
        return false;
    }

}
