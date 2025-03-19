package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase {
    private SparkFlex endeffectorMotor = new SparkFlex(Constants.endeffectorMotorId, MotorType.kBrushless);

    public EndEffectorSubsystem() {
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.idleMode(Constants.endEffectorIdleMode);
        endeffectorMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SparkFlex getEndAffectorMotor() {
        return endeffectorMotor;
    }

    public void setEndAffectorSpeed(double newSpeed) {
        endeffectorMotor.set(newSpeed);
    }

}
