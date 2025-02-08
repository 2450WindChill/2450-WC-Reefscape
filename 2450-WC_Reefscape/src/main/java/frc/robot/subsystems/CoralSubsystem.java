package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

  public SparkMax elevatorMotor = new SparkMax(1, MotorType.kBrushless);
  public SparkMax endeffectorMotor = new SparkMax(1, MotorType.kBrushless);
  
  public CoralSubsystem() {}

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}