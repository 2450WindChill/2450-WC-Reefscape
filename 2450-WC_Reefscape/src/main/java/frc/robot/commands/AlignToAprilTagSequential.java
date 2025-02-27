package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Camera;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTagSequential extends SequentialCommandGroup {

    VisionSubsystem m_visionSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;
    double m_strafeTarget;
    double m_approachTarget;

    public AlignToAprilTagSequential(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, double strafeTarget, double approachTarget, Enum<Camera> camera, BooleanSupplier stopSupplier) {
        addCommands(
            new SquareToAprilTag(visionSubsystem, drivetrainSubsystem, camera, stopSupplier),
            new StrafeToAprilTag(visionSubsystem, drivetrainSubsystem, strafeTarget, camera, stopSupplier),
            new ApproachAprilTag(visionSubsystem, drivetrainSubsystem, approachTarget, camera, stopSupplier)
        );
    }
}