package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class KillDriveCommands extends Command {
    DrivetrainSubsystem m_drivetrainSubsystem;

    public KillDriveCommands(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {

    }

    public void periodic() {

    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean isFinished) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, false, false);
    }
}
