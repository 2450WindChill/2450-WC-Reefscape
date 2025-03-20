package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class FullCoralIntake extends SequentialCommandGroup {
    public FullCoralIntake(CoralSubsystem coralSubsystem, EndEffectorSubsystem endEffectorSubsystem, double outSpeed, double inSpeed) {
        addCommands(
            new CoralIntakeStage1(coralSubsystem, endEffectorSubsystem, outSpeed)
            // new WaitCommand(0.2),
            // new CoralIntakeStage2(coralSubsystem, inSpeed)
        );
    }
}