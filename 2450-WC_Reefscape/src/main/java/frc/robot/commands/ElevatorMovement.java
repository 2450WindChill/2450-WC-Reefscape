package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ElevatorMovement extends Command {
    CoralSubsystem m_coralsubsystem;
    String m_direction;
    double m_speed;
    public ElevatorMovement(CoralSubsystem coralSubsystem, String direction, double speed) {
        m_coralsubsystem = coralSubsystem;
        m_direction = direction;
        m_speed = speed;
        addRequirements(m_coralsubsystem);
    }

    public void initialize() {
        if (m_direction == "up") {
            m_coralsubsystem.setElevatorSpeed(-m_speed);
        }
        if (m_direction == "down") {
            m_coralsubsystem.setElevatorSpeed(m_speed);
        }
    
    }

    public void execute() {
        System.out.println("Elevator movement direction: " + m_direction);
    }

    public void end(boolean interrupted) {
        m_coralsubsystem.setElevatorSpeed(0.0);
        System.out.println("Elevator Stopped");
    }

    public boolean isFinished() {
        if ((!m_coralsubsystem.getElevatorLowSwitch()) && (m_direction == "down")) {
            return true;
        } else {
            return false;
        }
    }
}
