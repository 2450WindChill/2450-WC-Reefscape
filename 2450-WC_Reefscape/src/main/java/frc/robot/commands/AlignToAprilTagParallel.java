package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTagParallel extends Command {

    VisionSubsystem m_VisionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;
    double m_strafeTarget;
    double m_approachTarget;

    double tolerance = Math.toRadians(1);
    double currAngle;
    double rotSpeed = 0;

    double strafeTolerance = 0.05;
    double strafeError;
    double strafeSpeed = 0;

    double approachTolerance = 0.05;
    double approachError;
    double approachSpeed = 0;

    public AlignToAprilTagParallel(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, double strafeTarget, double approachTarget) {
        m_VisionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_strafeTarget = strafeTarget;
        m_approachTarget = approachTarget;
    }

    public void initialize() {

    }

    public void execute() {

    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean isFinished) {

    }
}