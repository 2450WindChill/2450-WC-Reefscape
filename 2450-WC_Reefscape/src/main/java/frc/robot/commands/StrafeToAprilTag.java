package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class StrafeToAprilTag extends Command {

    ProfiledPIDController controller = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));

    VisionSubsystem m_VisionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;

    double tolerance = 0.05;

    double currError;

    double strafeSpeed;

    public StrafeToAprilTag(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        m_VisionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        // TODO: Convert to photonvision
        // controller.reset(m_VisionSubsystem.getAprilTagPoseToBot3d().getX());
        controller.setGoal(0);
        controller.setTolerance(tolerance);
    }

    public void execute() {
        // strafeSpeed = 0;

        // TODO: Convert to photonvision
        // currError = m_VisionSubsystem.getAprilTagPoseToBot3d().getX();
        
        // if (LimelightHelpers.getTV("limelight") && !controller.atGoal()) {
            strafeSpeed = -controller.calculate(currError);
        // }

        m_drivetrainSubsystem.drive(new Translation2d(0, strafeSpeed), 0, true, false);
        SmartDashboard.putNumber("strafeSpeed", strafeSpeed);
        SmartDashboard.putNumber("currError", currError);
    }

    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, false, false);
    }

    public boolean isFinished() {
        return false;

        // TODO: Convert to photonvision
        // return (!LimelightHelpers.getTV("limelight") || controller.atGoal());
    }
}