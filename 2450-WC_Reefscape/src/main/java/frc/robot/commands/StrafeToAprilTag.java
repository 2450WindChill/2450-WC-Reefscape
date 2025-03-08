package frc.robot.commands;

import java.util.Currency;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Camera;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class StrafeToAprilTag extends Command {

    ProfiledPIDController controller = new ProfiledPIDController(2, 0, 1, new Constraints(Constants.maxSpeed, 2));

    VisionSubsystem m_visionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;
    BooleanSupplier m_stopSupplier;
    double m_target;

    double tolerance = 0.04;
    double currError;
    double strafeSpeed = 0;

    int targetedID;

    Enum<Camera> m_camera;

    public StrafeToAprilTag(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, double target, Enum<Camera> camera, BooleanSupplier stopSupplier) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_target = target;
        m_camera = camera;
        m_stopSupplier = stopSupplier;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        if (m_camera == Camera.FRONT) {
            targetedID = m_visionSubsystem.getFrontAprilTagID();
        } else if (m_camera == Camera.BACK) {
            targetedID = m_visionSubsystem.getBackAprilTagID();
        }
        // Untested
        // controller.reset(m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getY() + Constants.VisionConstants.frontCameraLeftOffest);

        // Tested
        controller.reset(m_visionSubsystem.getFrontAprilTagPoseInRobotSpace().getY() + Constants.VisionConstants.frontCameraRightOffset);

        controller.setGoal(m_target);
        controller.setTolerance(tolerance);
    }

    public void execute() {
        // Untested but should stop the robot from tracking the wrong target
        // CHANGE ISFINISHED & INITIALIZE IF TRYING THIS LINE
        // currError = m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getY() + Constants.VisionConstants.frontCameraLeftOffest;
        
        // Tested
        currError = m_visionSubsystem.getFrontAprilTagPoseInRobotSpace().getY() + Constants.VisionConstants.frontCameraRightOffset;
        strafeSpeed = controller.calculate(currError);
        m_drivetrainSubsystem.drive(new Translation2d(0, strafeSpeed), 0, true, false);
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