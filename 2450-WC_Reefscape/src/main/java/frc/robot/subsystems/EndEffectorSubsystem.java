package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase {
    private SparkFlex endeffectorMotor = new SparkFlex(Constants.endeffectorMotorId, MotorType.kBrushless);

    // Beam Breaks
    private DigitalInput horizontalBeamBreak = new DigitalInput(Constants.horizontalBeamBreakID);
    private DigitalInput verticalBeamBreak = new DigitalInput(Constants.verticalBeamBreakID);

    public EndEffectorSubsystem() {
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.idleMode(Constants.endEffectorIdleMode);
        endeffectorMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {
        // Beam breaks
        SmartDashboard.putBoolean("Horizontal Break One", horizontalBeamBreak.get());
        SmartDashboard.putBoolean("Vertical Break One", verticalBeamBreak.get());
    }

    public SparkFlex getEndAffectorMotor() {
        return endeffectorMotor;
    }

    public void setEndAffectorSpeed(double newSpeed) {
        endeffectorMotor.set(newSpeed);
    }

    public DigitalInput getHorizontalBeamBreak() {
        return horizontalBeamBreak;
    }

    public DigitalInput getVerticalBeamBreak() {
        return verticalBeamBreak;
    }
}
