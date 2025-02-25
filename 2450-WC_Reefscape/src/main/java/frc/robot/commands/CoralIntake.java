// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntake extends Command {

  private final CoralSubsystem m_coralSubsystem;
  private final double m_speed;

  public CoralIntake(CoralSubsystem coralSubsystem, double speed) {
    m_coralSubsystem = coralSubsystem;
    m_speed = speed;
    
    addRequirements(coralSubsystem);
  }

  public void initialize() {
    m_coralSubsystem.getEndAffectorMotor().set(m_speed);
  }

  public void execute() {
  }

  public void end(boolean interrupted) {
    m_coralSubsystem.getEndAffectorMotor().set(0);
  }

  public boolean isFinished() {
    return !m_coralSubsystem.getBeamBreak().get();
  }
}
