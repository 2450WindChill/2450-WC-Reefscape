// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class CoralSubsystem extends SubsystemBase {
    private TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorId);
    private PIDController elevatorPidController = new PIDController(0.02, .001, 0);

    // Hall marks
    private DigitalInput bottomHallSensor = new DigitalInput(Constants.bottomHallSensorChannel);
    private DigitalInput intakeHallSensor = new DigitalInput(Constants.intakeHallSensorChannel);
    private DigitalInput topHallSensor = new DigitalInput(Constants.topHallSensorChannel);

    private CANdle candle = new CANdle(5);

    private Trigger intakeHallTrigger = new Trigger(bottomHallSensor::get);

    /** Creates a new ExampleSubsystem. */
    public CoralSubsystem() {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0.2; // no output for integrated error
        slot0Configs.kD = 0.3; // A velocity of 1 rps results in 0.1 V output
        elevatorMotor.getConfigurator().apply(slot0Configs);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeHallTrigger.onTrue(Commands.runOnce(() -> resetHeight(Constants.intakeHeight)));
    }

    public void setElevatorSpeed(double newSpeed) {
        final DutyCycleOut m_dutyCycle = new DutyCycleOut(0.0);
        elevatorMotor.setControl(m_dutyCycle.withOutput(newSpeed)
                .withLimitForwardMotion(!bottomHallSensor.get())
                .withLimitReverseMotion(!topHallSensor.get()));
    }

    public CANdle getCANdle() {
        return candle;
    }

    public void fireLEDS() {
        candle.configBrightnessScalar(1);
        FireAnimation fireAnimation = new FireAnimation(1, 0.4, 68, 0.5, 0.5);
        candle.animate(fireAnimation);
    }

    public void setAllianceColor() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            candle.setLEDs(255, 0, 0);
        } else {
            candle.setLEDs(0, 0, 255);
        }
    }

    public boolean getBottomHallSensor() {
        return bottomHallSensor.get();
    }

    // public boolean getElevatorHighSwitch() {
    // return elevatorHighSwitch.get();
    // }

    public void setPIDGoal(double position) {
        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        // set position to 10 rotations
        elevatorMotor.setControl(m_request.withPosition(position)
                .withLimitForwardMotion(!bottomHallSensor.get())
                .withLimitReverseMotion(!topHallSensor.get()));
    }

    public boolean goalReached(double goal) {
        double tolerance = 0.005;
        double currentPosition = elevatorMotor.getPosition().getValueAsDouble();
        System.err.println("Error: " + Math.abs(currentPosition - goal));
        if ((Math.abs(currentPosition - goal) < tolerance)) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        // Elevator info
        SmartDashboard.putNumber("Elevator Encoder", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Speed", elevatorMotor.get());

        // Hall sensors
        SmartDashboard.putBoolean("Bottom Hall Mark", bottomHallSensor.get());
        SmartDashboard.putBoolean("Intake Hall Mark", intakeHallSensor.get());
        SmartDashboard.putBoolean("Top Hall Mark", topHallSensor.get());

        resetHeight(Constants.intakeHeight);

        // if (!elevatorLowSwitch.get()) {
        // // zeroElevatorMotor();
        // resetEncoder();
        // }
        // if (!horizontalBeamBreak.get()) {
        // candle.setLEDs(0, 255, 0);
        // } else if (DriverStation.getAlliance().get() == Alliance.Red) {
        // candle.setLEDs(255, 0, 0);
        // } else {
        // candle.setLEDs(0, 0, 255);
        // }
    }

    public void resetHeight(double height) {
        elevatorMotor.setPosition(height);
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public TalonFX getElevatorMotorFx() {
        return elevatorMotor;
    }

    public PIDController getElevatorPIDcontroller() {
        return elevatorPidController;
    }

    public void resetEncoder() {
        elevatorMotor.setPosition(0);
    }

    public void zeroElevatorMotor() {
        elevatorMotor.set(0);
    }
}