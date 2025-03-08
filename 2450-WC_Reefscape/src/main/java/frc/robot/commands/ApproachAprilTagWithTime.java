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
import frc.robot.Constants.Camera;
// import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ApproachAprilTagWithTime extends Command {

    ProfiledPIDController controller = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));

    VisionSubsystem m_visionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;
    BooleanSupplier m_stopSupplier;
    double m_target;

    double tolerance = 0.05;
    double currError;
    double approachSpeed = 0.5;

    Timer timer = new Timer();
    double m_time = 0.0;

    int targetedID;

    Enum<Camera> m_camera;

    public ApproachAprilTagWithTime(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, double target, Enum<Camera> camera, BooleanSupplier stopSupplier, double time) {
        m_visionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_target = target;
        m_camera = camera;
        m_stopSupplier = stopSupplier;
        m_time = time;
        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        System.out.println("INITIALIZE APPROACH");
        timer.start();
        if (m_camera == Camera.FRONT) {
            targetedID = m_visionSubsystem.getFrontAprilTagID();
        } else if (m_camera == Camera.BACK) {
            targetedID = m_visionSubsystem.getBackAprilTagID();
        }
    }

    public void execute() {
        System.out.println("EXECUTE APPROACH");
        m_drivetrainSubsystem.drive(new Translation2d(-approachSpeed, 0), 0, true, false);
    }

    public void end(boolean interrupted) {
        System.out.println("END APPROACH");
        m_drivetrainSubsystem.drive(new Translation2d(), 0, false, false);
    }

    public boolean isFinished() {
        // if (m_stopSupplier.getAsBoolean() == true) {
        //     return true;
        // }
        System.out.println();
        if (timer.hasElapsed(m_time)) {
            timer.stop();
            timer.reset();  
            return true;   
          } else {
            return false;
          }
    }
}