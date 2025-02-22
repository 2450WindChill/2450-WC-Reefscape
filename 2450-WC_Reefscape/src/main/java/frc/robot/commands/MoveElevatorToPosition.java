package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;

import javax.swing.text.html.parser.ContentModel;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveElevatorToPosition extends Command {
  private final CoralSubsystem m_coralSubsystem;
  private int m_target;

  public MoveElevatorToPosition(CoralSubsystem coralSubsystem, int target) {
    m_coralSubsystem = coralSubsystem;
    m_target = target;
    addRequirements(m_coralSubsystem);
  }

  public void initialize() {
    m_coralSubsystem.setPIDGoal(m_target);
  }

  public void execute() {
  }

  public void end(boolean interrupted) {
    m_coralSubsystem.getElevatorMotorFx().set(0);
  }

  public boolean isFinished() {
    return m_coralSubsystem.goalReached(m_target);
  }
}
