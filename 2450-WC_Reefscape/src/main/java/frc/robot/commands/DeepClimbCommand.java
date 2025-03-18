package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DeepClimbSubsystem;

import java.util.function.BooleanSupplier;

import javax.swing.text.html.parser.ContentModel;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DeepClimbCommand extends Command {
    private final DeepClimbSubsystem m_DeepClimbSubsystem;
    private double m_targetOne;
    private double m_targetTwo;
    BooleanSupplier m_overrideSupplier;
    double climbTolerance = 0.01;

    double absEncoderOneSpeed;
    double absEncoderTwoSpeed;

     ProfiledPIDController absControllerOne = new ProfiledPIDController(1, 0, 0, new Constraints(1, 2));
     ProfiledPIDController absControllerTwo = new ProfiledPIDController(0.1, 0, 0, new Constraints(1, 2));

    public DeepClimbCommand(DeepClimbSubsystem deepClimbSubsystem, double targetOne, double targetTwo, BooleanSupplier overrideSupplier) {
        m_DeepClimbSubsystem = deepClimbSubsystem;
        m_targetOne = targetOne;
        m_targetTwo = targetTwo;
        m_overrideSupplier = overrideSupplier;
        addRequirements(m_DeepClimbSubsystem);
    }

    public void initialize() {
        double initialPositionOne = m_DeepClimbSubsystem.absoluteEncoderOne.get();
        double initialPositionTwo = m_DeepClimbSubsystem.absoluteEncoderTwo.get();
        absControllerOne.reset(initialPositionOne);
        absControllerTwo.reset(initialPositionTwo);

        absControllerOne.setTolerance(climbTolerance);
        absControllerTwo.setTolerance(climbTolerance);

        absControllerOne.setGoal(m_targetOne);
        absControllerTwo.setGoal(m_targetOne);
        absControllerTwo.setGoal(m_targetTwo);
    }

    public void execute() {
        double currentPositionOne = m_DeepClimbSubsystem.absoluteEncoderOne.get();
        double currentPositionTwo = m_DeepClimbSubsystem.absoluteEncoderTwo.get();

        absEncoderOneSpeed = Math.min(absControllerOne.calculate(currentPositionOne), 0.2);
        absEncoderTwoSpeed = Math.min(absControllerOne.calculate(currentPositionTwo), 0.2);

        m_DeepClimbSubsystem.setClimbMotorOne(absEncoderOneSpeed);
        m_DeepClimbSubsystem.setClimbMotorTwo(absEncoderTwoSpeed);
    }

    public void end(boolean interrupted) {
        m_DeepClimbSubsystem.getClimbMotorOne().set(0);
        m_DeepClimbSubsystem.getClimbMotorTwo().set(0);
    }

    public boolean isFinished() {
        return (absControllerOne.atGoal() && absControllerTwo.atGoal() || m_overrideSupplier.getAsBoolean());
    }
}
