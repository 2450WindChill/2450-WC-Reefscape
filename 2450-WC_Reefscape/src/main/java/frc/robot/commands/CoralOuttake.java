// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CoralOuttake extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CoralSubsystem m_subsystem;

  private final double m_speed;

  public CoralOuttake(CoralSubsystem subsystem, double speed) {
    m_subsystem = subsystem;
    m_speed = speed;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.getEndAffectorMotor().set(m_speed);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_subsystem.getCANdle().setLEDs(255, 0, 0);
    } else {
      m_subsystem.getCANdle().setLEDs(0, 0, 255);
    }
    m_subsystem.getEndAffectorMotor().set(0);
  }

  @Override
  public boolean isFinished() {
    return m_subsystem.getHorizontalBeamBreak().get();
  }
}