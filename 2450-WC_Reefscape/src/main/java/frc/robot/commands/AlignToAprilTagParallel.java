package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Camera;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTagParallel extends Command {

    VisionSubsystem m_visionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;
    double m_strafeTarget;
    double m_approachTarget;
    boolean isInverted;

    ProfiledPIDController rotController = new ProfiledPIDController(6, 0, 0, new Constraints(Constants.maxAngularVelocity, 4));
    ProfiledPIDController strafeController = new ProfiledPIDController(2.5, 0, 0, new Constraints(Constants.maxSpeed, 2));
    ProfiledPIDController approachController = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));

    double rotTolerance = Math.toRadians(1);
    double currAngle;
    double rotSpeed = 0;

    double strafeTolerance = 0.05;
    double strafeError;
    double strafeSpeed = 0;

    double approachTolerance = 0.05;
    double approachError;
    double approachSpeed = 0;

    int targetedID;

    Enum<Camera> m_camera;

    public AlignToAprilTagParallel(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, double strafeTarget, double approachTarget, Enum<Camera> camera) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_strafeTarget = strafeTarget;
        m_approachTarget = approachTarget;
    }

    public void initialize() {
        targetedID = m_visionSubsystem.getFrontAprilTagID();
        double currentReading = m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getRotation().getRadians();
        rotController.reset(currentReading);
        if (currentReading > 0) {
            rotController.setGoal(Math.PI);
            isInverted = false;
        } else {
            rotController.setGoal(-Math.PI);
            isInverted = true;
        }
        rotController.setTolerance(rotTolerance);

        strafeController.reset(m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getY() + Constants.VisionConstants.frontCameraLeftOffest);
        strafeController.setGoal(m_strafeTarget);
        strafeController.setTolerance(strafeTolerance);

        approachController.reset(m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getX() + Constants.VisionConstants.frontCameraForwardOffest);
        approachController.setGoal(m_approachTarget);
        approachController.setTolerance(approachTolerance);
    }

    public void execute() {
        if (!rotController.atGoal()) {
            currAngle = m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getRotation().getRadians();
            if (isInverted) {
                currAngle = -Math.abs(currAngle);
            } else {
                currAngle = Math.abs(currAngle);
            }
            rotSpeed = rotController.calculate(currAngle);
        } else {
            rotSpeed = 0;
        }

        if (!strafeController.atGoal()) {
            strafeError = m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getY() + Constants.VisionConstants.frontCameraLeftOffest;
            strafeSpeed = strafeController.calculate(strafeError);
        } else {
            strafeSpeed = 0;
        }

        if (!approachController.atGoal()) {
            approachError = m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getX() + Constants.VisionConstants.frontCameraForwardOffest;
            approachSpeed = approachController.calculate(approachError);
        } else {
            approachSpeed = 0;
        }

        m_drivetrainSubsystem.drive(new Translation2d(approachSpeed, strafeSpeed), rotSpeed, true, false);
    }

    public boolean isFinished() {
        return ((m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID) == new Pose2d()) || (rotController.atGoal() && strafeController.atGoal() && approachController.atGoal()));
    }

    public void end(boolean isFinished) {
        m_drivetrainSubsystem.drive(new Translation2d(0, 0), 0, true, false);
    }
}