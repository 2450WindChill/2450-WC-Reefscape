package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

    double driveTolerance = 0.05;
    double rotTolerance = Math.toRadians(3);

    double xTarget;
    double yTarget;
    double rotTarget;

    double xSpeed;
    double ySpeed;
    double rotSpeed;

    public MoveToPose(DrivetrainSubsystem drivetrainSubsystem, Pose2d target, BooleanSupplier overrideSupplier) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_overrideSupplier = overrideSupplier;
        xTarget = target.getX();
        yTarget = target.getY();
        rotTarget = target.getRotation().getRadians();
    }

    public void initialize() {
        Pose2d initialPose = m_drivetrainSubsystem.getThisPose();
        xController.reset(initialPose.getX());
        yController.reset(initialPose.getY());
        rotController.reset(initialPose.getRotation().getRadians());

        xController.setTolerance(driveTolerance);
        yController.setTolerance(driveTolerance);
        rotController.setTolerance(rotTolerance);

        xController.setGoal(xTarget);
        yController.setGoal(yTarget);
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
        SmartDashboard.putNumber("Current pose X: ", currPose.getX());
        SmartDashboard.putNumber("Current pose Y: ", currPose.getY());
        SmartDashboard.putNumber("Current pose Rot: ", currPose.getRotation().getDegrees());

        if (!xController.atGoal()) {
            xSpeed = xController.calculate(currPose.getX());
        } else {
            xSpeed = 0;
        }

        if (!yController.atGoal()) {
            ySpeed = yController.calculate(currPose.getY());
        } else {
            ySpeed = 0;
        }

        if (!rotController.atGoal()) {
            rotSpeed = rotController.calculate(currPose.getRotation().getRadians());
        } else {
            rotSpeed = 0;
        }

        m_drivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, false, false);

        System.out.println("X at goal: " + xController.atGoal() + " Y at goal: " + yController.atGoal() + " Rot at goal: " + rotController.atGoal());
    }

    // TODO Change is finished from testing with return false to actual isFinished
    public boolean isFinished() {
        // return false;
        return (xController.atGoal() && yController.atGoal() && rotController.atGoal()) || m_overrideSupplier.getAsBoolean();
    }

    public void end(boolean isFinished) {
        m_drivetrainSubsystem.drive(new Translation2d(0, 0), 0, false, false);
    }
}
