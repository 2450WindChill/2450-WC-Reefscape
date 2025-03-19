package frc.robot.commands;

import java.util.function.BooleanSupplier;

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
    BooleanSupplier m_stopSupplier;
    double m_target;

    double tolerance = 0.05;
    double currError;
    double approachSpeed = 0;

    int targetedID;

    Enum<Camera> m_camera;

    public ApproachAprilTag(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, double target, Enum<Camera> camera, BooleanSupplier stopSupplier) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_target = target;
        m_camera = camera;
        m_stopSupplier = stopSupplier;

        addRequirements(m_visionSubsystem, m_drivetrainSubsystem);
    }

    public void initialize() {
        if (m_camera == Camera.FRONT) {
            targetedID = m_visionSubsystem.getFrontAprilTagID();
        } else if (m_camera == Camera.BACK) {
            targetedID = m_visionSubsystem.getBackAprilTagID();
        }
        // Untested
        // controller.reset(m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getX() + Constants.VisionConstants.frontCameraForwardOffest);

        //Tested 
        controller.reset(m_visionSubsystem.getFrontAprilTagPoseInRobotSpace().getX() + Constants.VisionConstants.frontCameraForwardOffset);
        controller.setGoal(m_target);
        controller.setTolerance(tolerance);
    }

    public void execute() {
        // Untested but should stop the robot from tracking the wrong target
        // CHANGE ISFINISHED & INITIALIZE IF TRYING THIS LINE
        // currError = m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getX() + Constants.VisionConstants.frontCameraForwardOffest;

        // Tested
        currError = m_visionSubsystem.getFrontAprilTagPoseInRobotSpace().getX() + Constants.VisionConstants.frontCameraForwardOffset;

        approachSpeed = controller.calculate(currError);
        m_drivetrainSubsystem.drive(new Translation2d(approachSpeed, 0), 0, true, false);
    }

    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, false, false);
    }

    public boolean isFinished() {
        if (m_stopSupplier.getAsBoolean() == true) {
            return true;
        }
        return (!m_visionSubsystem.frontCameraHasTarget() || controller.atGoal());
    }
}