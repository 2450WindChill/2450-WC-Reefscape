package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class BopAlgae extends Command {
    CoralSubsystem m_coralSubsystem;
    DoubleSupplier m_forwardSpeed;
    DoubleSupplier m_backwardSpeed;

    String m_direction;

    public BopAlgae(CoralSubsystem coralSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier backwardSpeed) {
        m_coralSubsystem = coralSubsystem;
        m_backwardSpeed = backwardSpeed;
        m_forwardSpeed = forwardSpeed;

        addRequirements(m_coralSubsystem);
    }

    public void initialize() {
    }

    public void execute() {
        double fS = (m_backwardSpeed.getAsDouble() - m_forwardSpeed.getAsDouble()) * 0.5;
        m_coralSubsystem.setEndAffectorSpeed(fS);
        ;
    }

    public void end(boolean interrupted) {
        m_coralSubsystem.setEndAffectorSpeed(0);
    }

    public boolean isFinished() {
        return false;
    }

}
