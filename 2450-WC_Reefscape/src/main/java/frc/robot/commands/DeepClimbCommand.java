package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DeepClimbSubsystem;

import javax.swing.text.html.parser.ContentModel;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DeepClimbCommand extends Command {
    private final DeepClimbSubsystem m_DeepClimbSubsystem;
    private int m_targetOne;
    private int m_targetTwo;

     ProfiledPIDController absControllerOne = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));
     ProfiledPIDController absControllerTwo = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));

    public DeepClimbCommand(DeepClimbSubsystem deepClimbSubsystem, int targetOne, int targetTwo) {
        m_DeepClimbSubsystem = deepClimbSubsystem;
        m_targetOne = targetOne;
        m_targetTwo = targetTwo;
        addRequirements(m_DeepClimbSubsystem);
    }

    public void initialize() {
        m_DeepClimbSubsystem.setPIDGoal(m_targetOne, m_targetTwo);
        absControllerOne.setGoal(m_targetOne);
        absControllerTwo.setGoal(m_targetOne);
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
        m_DeepClimbSubsystem.getClimbMotorOne().set(0);
        m_DeepClimbSubsystem.getClimbMotorTwo().set(0);
    }

    public boolean isFinished() {
        return (m_DeepClimbSubsystem.goalReachedClimbMotorOne(m_targetOne)
                && m_DeepClimbSubsystem.goalReachedClimbMotorTwo(m_targetTwo));
    }
}
