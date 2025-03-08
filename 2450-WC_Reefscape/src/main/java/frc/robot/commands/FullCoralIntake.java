package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralSubsystem;

public class FullCoralIntake extends SequentialCommandGroup {
    public FullCoralIntake(CoralSubsystem coralSubsystem, double outSpeed, double inSpeed) {
        addCommands(
            new CoralIntakeStage1(coralSubsystem, outSpeed)
            // new WaitCommand(0.2),
            // new CoralIntakeStage2(coralSubsystem, inSpeed)
        );
    }
}