package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DeepClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ClimberMovement extends Command {
    DeepClimbSubsystem m_DeepClimbSubsystem;
    String m_direction;
    double m_speed;
    public ClimberMovement(DeepClimbSubsystem deepClimbSubsystem, String direction, double speed) {
        m_DeepClimbSubsystem = deepClimbSubsystem;
        m_direction = direction;
        m_speed = speed;
        addRequirements(m_DeepClimbSubsystem);
    }

    public void initialize() {
        if (m_direction == "in") {
            m_DeepClimbSubsystem.setClimberMotors(m_speed);
        }
        if (m_direction == "out") {
            m_DeepClimbSubsystem.setClimberMotors(-m_speed);
        }
    
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
        m_DeepClimbSubsystem.setClimberMotors(0.0);
    }

    public boolean isFinished() {
        return false;
    }
}