package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class BopAlgae extends Command {
    EndEffectorSubsystem m_EndEffectorSubsystem;
    LEDSubsystem m_ledSubsystem;
    double m_Speed;

    String m_direction;

    public BopAlgae(EndEffectorSubsystem endEffectorSubsystem, LEDSubsystem ledSubsystem, double speed) {
        m_EndEffectorSubsystem = endEffectorSubsystem;
        m_Speed = speed;
        m_ledSubsystem = ledSubsystem;

        addRequirements(m_EndEffectorSubsystem);
    }

    public void initialize() {
        m_ledSubsystem.blinkAllianceColor();
    }

    public void execute() {
        m_EndEffectorSubsystem.setEndAffectorSpeed(m_Speed);
    }

    public void end(boolean interrupted) {
        m_EndEffectorSubsystem.setEndAffectorSpeed(0);
        m_ledSubsystem.setAllianceColor();
    }

    public boolean isFinished() {
        return false;
    }
}