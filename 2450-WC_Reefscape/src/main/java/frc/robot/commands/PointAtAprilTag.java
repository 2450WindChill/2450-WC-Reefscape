package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PointAtAprilTag extends Command {

    ProfiledPIDController controller = new ProfiledPIDController(5, 0, 0, new Constraints(Constants.maxAngularVelocity, 4));

    VisionSubsystem m_visionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;

    double tolerance = Math.toRadians(5);
    double currAngle;
    double rotSpeed = 0;

    public PointAtAprilTag(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        controller.reset(m_visionSubsystem.getFrontAprilTagYaw().getRadians());
        controller.setGoal(0);
        controller.setTolerance(tolerance);
    }

    public void execute() {
        currAngle = m_visionSubsystem.getFrontAprilTagYaw().getRadians();
        rotSpeed = -controller.calculate(currAngle);
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