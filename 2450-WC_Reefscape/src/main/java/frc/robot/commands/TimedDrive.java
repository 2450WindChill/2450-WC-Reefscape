package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TimedDrive extends Command {

    Timer timer = new Timer();
    DrivetrainSubsystem m_drivetrainSubsystem;
    Translation2d m_translation;
    double m_duration;

    public TimedDrive(DrivetrainSubsystem drivetrainSubsystem, Translation2d translation, double duration) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_translation = translation;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        timer.reset();
        timer.start();
    }

    public void execute() {
        m_drivetrainSubsystem.drive(m_translation, 0, false, false);
    }

    public boolean isFinished() {
        return timer.get() >= m_duration;
    }

    public void end(boolean isFinished) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, false, false);
    }
}
