package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Camera;
// import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ApproachAprilTag extends Command {

    ProfiledPIDController controller = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));

    VisionSubsystem m_visionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;
    double m_target;

    double tolerance = 0.05;
    double currError;
    double approachSpeed = 0;

    int targetedID;

    Enum<Camera> m_camera;

    public ApproachAprilTag(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, double target, Enum<Camera> camera) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_target = target;
        m_camera = camera;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        if (m_camera == Camera.FRONT) {
            targetedID = m_visionSubsystem.getFrontAprilTagID();
        } else if (m_camera == Camera.BACK) {
            targetedID = m_visionSubsystem.getBackAprilTagID();
        }
        controller.reset(m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getX() + Constants.VisionConstants.frontCameraForwardOffest);
        controller.setGoal(m_target);
        controller.setTolerance(tolerance);
    }

    public void execute() {
        currError = m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getX() + Constants.VisionConstants.frontCameraForwardOffest;
        approachSpeed = controller.calculate(currError);
        m_drivetrainSubsystem.drive(new Translation2d(approachSpeed, 0), 0, true, false);
    }

    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, false, false);
    }

    public boolean isFinished() {
        return ((m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID) == new Pose2d()) || controller.atGoal());
    }
}