package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeepClimbSubsystem extends SubsystemBase {
  private TalonFX climbMotorOne = new TalonFX(Constants.climbMotorOneId);
  private TalonFX climbMotorTwo = new TalonFX(Constants.climbMotorTwoId);

  public DeepClimbSubsystem() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Motor One Encoder", climbMotorOne.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber Motor Two Encoder", climbMotorTwo.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
  }

  public void setPIDGoal(int positionOne, int positionTwo) {
    // create a position closed-loop request, voltage output, slot 0 configs
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    // set position to 10 rotations
    climbMotorOne.setControl(m_request.withPosition(positionOne));
    climbMotorTwo.setControl(m_request.withPosition(positionTwo));
  }

  public TalonFX getClimbMotorOne() {
    return climbMotorOne;
  }

  public TalonFX getClimbMotorTwo() {
    return climbMotorTwo;
  }

  public void setClimberMotors(double speed) {
    climbMotorOne.set(speed);
    climbMotorTwo.set(-speed);
}

  public boolean goalReachedClimbMotorOne(int goal) {
    double tolerance = 0.05;
    double currentPosition = climbMotorOne.getPosition().getValueAsDouble();
    if ((Math.abs(currentPosition - goal) < tolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean goalReachedClimbMotorTwo(int goal) {
    double tolerance = 0.05;
    double currentPosition = climbMotorTwo.getPosition().getValueAsDouble();
    if ((Math.abs(currentPosition - goal) < tolerance)) {
      return true;
    } else {
      return false;
    }
  }
}