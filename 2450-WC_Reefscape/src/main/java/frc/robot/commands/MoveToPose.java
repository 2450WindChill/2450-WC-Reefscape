package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveToPose extends Command {

    DrivetrainSubsystem m_drivetrainSubsystem;

    ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));
    ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));
    ProfiledPIDController rotController = new ProfiledPIDController(6, 0, 0, new Constraints(Constants.maxAngularVelocity, 4));

    BooleanSupplier m_overrideSupplier;

    double driveTolerance = 0.02;
    double rotTolerance = Math.toRadians(2);

    double xTarget;
    double yTarget;
    double rotTarget;
    Timer timer_seconds = new Timer();

    double xSpeed;
    double ySpeed;
    double rotSpeed;
    double m_time;

    boolean rotIsInverted;

    public MoveToPose(DrivetrainSubsystem drivetrainSubsystem, Pose2d target, BooleanSupplier overrideSupplier, double time) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_overrideSupplier = overrideSupplier;
        xTarget = target.getX();
        yTarget = target.getY();
        rotTarget = target.getRotation().getRadians();
        m_time = time;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        timer_seconds.restart();
        Pose2d initialPose = m_drivetrainSubsystem.getThisPose();
        xController.reset(initialPose.getX());
        yController.reset(initialPose.getY());
        rotController.reset(initialPose.getRotation().getRadians());

        xController.setTolerance(driveTolerance);
        yController.setTolerance(driveTolerance);
        rotController.setTolerance(rotTolerance);

        xController.setGoal(xTarget);
        yController.setGoal(yTarget);

        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.setGoal(rotTarget);

        SmartDashboard.putNumber("Initial pose X: ", initialPose.getX());
        SmartDashboard.putNumber("Initial pose Y: ", initialPose.getY());
        SmartDashboard.putNumber("Initial pose Rot: ", initialPose.getRotation().getDegrees());

        SmartDashboard.putNumber("Target pose X: ", xTarget);
        SmartDashboard.putNumber("Target pose Y: ", yTarget);
        SmartDashboard.putNumber("Target pose Rot: ", rotTarget);
    }

    public void execute() {
        Pose2d currPose = m_drivetrainSubsystem.getThisPose();
        
        xSpeed = xController.calculate(currPose.getX());
        ySpeed = yController.calculate(currPose.getY());
        rotSpeed = -rotController.calculate(currPose.getRotation().getRadians());

        m_drivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, false, false);

        System.out.println("X at goal: " + xController.atGoal() + " Y at goal: " + yController.atGoal() + " Rot at goal: " + rotController.atGoal());
    }

    public boolean isFinished() {
        if (timer_seconds.get() >= m_time) {
            return true;
        }
        // return false;
        return (xController.atGoal() && yController.atGoal() && rotController.atGoal()) || m_overrideSupplier.getAsBoolean();
    }

    public void end(boolean isFinished) {
        m_drivetrainSubsystem.drive(new Translation2d(0, 0), 0, false, false);
    }
}
