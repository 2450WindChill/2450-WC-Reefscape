// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntakeStage1 extends Command {

  private final CoralSubsystem m_coralSubsystem;
  private final double m_speed;
  private DigitalInput beamBreak;

  private boolean currentBeamBreakState;
  private int stateChanges;

  public CoralIntakeStage1(CoralSubsystem coralSubsystem, double speed) {
    m_coralSubsystem = coralSubsystem;
    m_speed = speed;
    beamBreak = coralSubsystem.getVerticalBeamBreak();

    currentBeamBreakState = beamBreak.get();

    addRequirements(coralSubsystem);
  }

  public void initialize() {
    stateChanges = 0;
    m_coralSubsystem.getEndAffectorMotor().set(m_speed);
  }

  public void execute() {
    if (beamBreak.get() != currentBeamBreakState) {
      stateChanges += 1;
      currentBeamBreakState = beamBreak.get();
    }
  }

  public void end(boolean interrupted) {
    System.out.println("Coral intake done");
    m_coralSubsystem.getEndAffectorMotor().set(0);
    m_coralSubsystem.fireLEDS();

  }

  public boolean isFinished() {
    return stateChanges >= 2;
  }
}
