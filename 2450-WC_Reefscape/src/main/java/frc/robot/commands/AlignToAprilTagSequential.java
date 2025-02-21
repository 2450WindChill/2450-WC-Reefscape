package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTagSequential extends SequentialCommandGroup {

    VisionSubsystem m_VisionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;
    double m_strafeTarget;
    double m_approachTarget;

    public AlignToAprilTagSequential(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, double strafeTarget, double approachTarget) {
        m_VisionSubsystem = visionSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_strafeTarget = strafeTarget;
        m_approachTarget = approachTarget;

        addCommands(
            new SquareToAprilTag(m_VisionSubsystem, drivetrainSubsystem),
            new StrafeToAprilTag(m_VisionSubsystem, drivetrainSubsystem, m_strafeTarget),
            new ApproachAprilTag(m_VisionSubsystem, drivetrainSubsystem, m_approachTarget)
        );
    }
}