package frc.robot.commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;


public class SwerveMicroAdjustCommand extends Command {
    private final DrivetrainSubsystem m_DrivetrainSubsystem;
    private final XboxController m_XboxController;
    private final double m_adjustSpeed;

    public SwerveMicroAdjustCommand(DrivetrainSubsystem drivetrainSubsystem, XboxController xboxController, double adjustSpeed) {
        m_DrivetrainSubsystem = drivetrainSubsystem;
        m_XboxController = xboxController;
        m_adjustSpeed = adjustSpeed;
        addRequirements(m_DrivetrainSubsystem);
    }

    @Override
    public void execute() {
        int pov = m_XboxController.getPOV();
        double forwardSpeed = 0.0;
        double strafeSpeed = 0.0;

        // Check the POV value and set speeds accordingly.
        if (pov == 0) {
            // D-pad up: move forward (robot's front)
            forwardSpeed = m_adjustSpeed;
        } else if (pov == 180) {
            // D-pad down: move backward
            forwardSpeed = -m_adjustSpeed;
        } else if (pov == 90) {
            // D-pad right: strafe right
            strafeSpeed = m_adjustSpeed;
        } else if (pov == 270) {
            // D-pad left: strafe left
            strafeSpeed = -m_adjustSpeed;
        }
        
        // Drive with the given speeds, no rotation.
        // The last parameter 'true' indicates robot-oriented control.
        m_DrivetrainSubsystem.drive(new Translation2d(forwardSpeed, strafeSpeed), 0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all movement when the command ends.
        m_DrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        // This command runs until it is interrupted.
        return false;
    }
}