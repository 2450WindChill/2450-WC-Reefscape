package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

import javax.swing.text.html.parser.ContentModel;
import javax.xml.stream.events.EndDocument;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/** An example command that uses an example subsystem. */
public class MoveElevatorToPosition extends Command {
  private final LEDSubsystem m_ledSubsystem;
  private final EndEffectorSubsystem m_endEffectorSubsystem;
  private final CoralSubsystem m_coralSubsystem;
  private double m_target;

  public MoveElevatorToPosition(CoralSubsystem coralSubsystem, EndEffectorSubsystem endEffectorSubsystem, LEDSubsystem ledsubsystem, double target) {
    m_ledSubsystem = ledsubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;
    m_coralSubsystem = coralSubsystem;
    m_target = target;
    addRequirements(m_coralSubsystem);
  }

  public void initialize() {
    m_coralSubsystem.setPIDGoal(m_target);
    m_ledSubsystem.setAllianceColor();
  }

  public void execute() {
  }

  public void end(boolean interrupted) {
    m_coralSubsystem.getElevatorMotorFx().set(0);

    if (!m_endEffectorSubsystem.getVerticalBeamBreak().get()) {
      m_ledSubsystem.setAllianceColor();
    } else if (m_target == Constants.intakeHeight) {
      m_ledSubsystem.setLEDColor(0, 255, 0, 0);
    } else if (m_target == Constants.L1Height || m_target == Constants.L2Height || m_target == Constants.L3Height) {
      m_ledSubsystem.setLEDColor(255, 0, 255, 0);
    } else {
      m_ledSubsystem.setAllianceColor();
    }
  }

  public boolean isFinished() {
    return m_coralSubsystem.goalReached(m_target) || !m_endEffectorSubsystem.getVerticalBeamBreak().get();
  }
}


