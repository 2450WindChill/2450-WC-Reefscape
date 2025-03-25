// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/** An example command that uses an example subsystem. */
public class CoralOuttake extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final EndEffectorSubsystem m_endEffectorSubsystem;
  private final LEDSubsystem m_ledSubsystem;

  private final double m_speed;

  public CoralOuttake(EndEffectorSubsystem endEffectorSubsystem, LEDSubsystem ledSubsystem, double speed) {
    m_endEffectorSubsystem = endEffectorSubsystem;
    m_ledSubsystem = ledSubsystem;
    m_speed = speed;

    addRequirements(m_endEffectorSubsystem, m_ledSubsystem);
  }

  @Override
  public void initialize() {
    m_endEffectorSubsystem.getEndAffectorMotor().set(m_speed);
    m_ledSubsystem.setLEDSBlinking(0, 0, 0, 255);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Coral outtake done");
    m_ledSubsystem.setAllianceColor();
    m_endEffectorSubsystem.getEndAffectorMotor().set(0);
  }

  @Override
  public boolean isFinished() {
    return m_endEffectorSubsystem.getHorizontalBeamBreak().get();
  }
}