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

public class CoralIntakeStage2 extends Command {

  private final CoralSubsystem m_coralSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;
  private final double m_speed;
  private DigitalInput beamBreak;

  private boolean currentBeamBreakState;

  public CoralIntakeStage2(CoralSubsystem coralSubsystem, EndEffectorSubsystem endEffectorSubsystem, double speed) {
    m_coralSubsystem = coralSubsystem;
    m_EndEffectorSubsystem = endEffectorSubsystem;
    m_speed = speed;
    beamBreak = coralSubsystem.getVerticalBeamBreak();

    currentBeamBreakState = beamBreak.get();

    addRequirements(endEffectorSubsystem);
  }

  public void initialize() {
    m_EndEffectorSubsystem.getEndAffectorMotor().set(-m_speed);
  }

  public void execute() {
    
  }

  public void end(boolean interrupted) {
    System.out.println("Coral intake stage 2 done");
    m_EndEffectorSubsystem.getEndAffectorMotor().set(0);
    m_coralSubsystem.fireLEDS();

  }

  public boolean isFinished() {
    return (beamBreak.get() != currentBeamBreakState);
  }
}
