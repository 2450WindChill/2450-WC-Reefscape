// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntake extends Command {

  private final EndEffectorSubsystem m_endEffectorSubsystem;
  private final double m_speed;
  private DigitalInput verticalBeamBreak;

  private boolean currentBeamBreakState;
  private int stateChanges;

  public CoralIntake(EndEffectorSubsystem endEffectorSubsystem, double speed) {
    m_speed = speed;
    m_endEffectorSubsystem = endEffectorSubsystem;
    verticalBeamBreak = endEffectorSubsystem.getVerticalBeamBreak();

    currentBeamBreakState = verticalBeamBreak.get();

    addRequirements(m_endEffectorSubsystem);
  }

  public void initialize() {
    stateChanges = 0;
    m_endEffectorSubsystem.getEndAffectorMotor().set(m_speed);
  }

  public void execute() {
    if (verticalBeamBreak.get() != currentBeamBreakState) {
      System.err.println("State Change from " + currentBeamBreakState + " to " + verticalBeamBreak.get());
      stateChanges += 1;
      currentBeamBreakState = verticalBeamBreak.get();
    }
  }

  public void end(boolean interrupted) {
    System.out.println("Coral intake done");
    m_endEffectorSubsystem.getEndAffectorMotor().set(0);
  }

  public boolean isFinished() {
    return stateChanges >= 2;
  }
}
