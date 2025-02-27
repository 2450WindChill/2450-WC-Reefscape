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
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SquareToAprilTag extends Command {

    ProfiledPIDController controller = new ProfiledPIDController(6, 0, 0, new Constraints(Constants.maxAngularVelocity, 4));

    VisionSubsystem m_visionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;
    BooleanSupplier m_stopSupplier;

    double tolerance = Math.toRadians(1);
    double currAngle;
    double rotSpeed = 0;

    boolean isInverted;

    int targetedID;

    Enum<Camera> m_camera;

    public SquareToAprilTag(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, Enum<Camera> camera, BooleanSupplier stopSupplier) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
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
        // double currentReading = m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getRotation().getRadians();

        // Tested
        double currentReading = m_visionSubsystem.getFrontAprilTagPoseInRobotSpace().getRotation().getRadians();

        controller.reset(currentReading);
        if (currentReading > 0) {
            controller.setGoal(Math.PI);
            isInverted = false;
        } else {
            controller.setGoal(-Math.PI);
            isInverted = true;
        }
        controller.setTolerance(tolerance);
    }

    public void execute() {
        // Untested but should stop the robot from tracking the wrong target
        // CHANGE ISFINISHED & INITIALIZE IF TRYING THIS LINE
        // currAngle = m_visionSubsystem.getAprilTagPoseInRobotSpace(targetedID).getRotation().getRadians();

        //Tested
        currAngle = m_visionSubsystem.getFrontAprilTagPoseInRobotSpace().getRotation().getRadians();


        if (isInverted) {
            currAngle = -Math.abs(currAngle);
        } else {
            currAngle = Math.abs(currAngle);
        }
        rotSpeed = controller.calculate(currAngle);
        m_drivetrainSubsystem.drive(new Translation2d(), rotSpeed, true, false);
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