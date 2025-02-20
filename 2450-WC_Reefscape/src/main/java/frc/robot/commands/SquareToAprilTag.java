package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SquareToAprilTag extends Command {

    ProfiledPIDController controller = new ProfiledPIDController(6, 0, 0, new Constraints(Constants.maxAngularVelocity, 4));

    VisionSubsystem m_visionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;

    double tolerance = Math.toRadians(1);
    double currAngle;
    double rotSpeed = 0;

    boolean isInverted;

    public SquareToAprilTag(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
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
        currAngle = m_visionSubsystem.getFrontAprilTagPoseInRobotSpace().getRotation().getRadians();
        if (isInverted) {
            currAngle = -Math.abs(currAngle);
        } else {
            currAngle = Math.abs(currAngle);
        }
        rotSpeed = controller.calculate(currAngle);
        m_drivetrainSubsystem.drive(new Translation2d(), rotSpeed, true, false);
        SmartDashboard.putBoolean("At Rotation Goal", controller.atGoal());
    }

    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, false, false);
    }

    public boolean isFinished() {
        return (!m_visionSubsystem.frontCameraHasTarget() || controller.atGoal());
    }
}