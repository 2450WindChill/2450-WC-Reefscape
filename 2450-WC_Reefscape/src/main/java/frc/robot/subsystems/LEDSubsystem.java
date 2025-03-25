package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private CANdle candle = new CANdle(5);

    public LEDSubsystem() {
    }

    @Override
    public void periodic() {
    }

    public CANdle getCANdle() {
        return candle;
    }

    public void fireLEDS() {
        candle.configBrightnessScalar(1);
        FireAnimation fireAnimation = new FireAnimation(1, 0.4, 68, 0.5, 0.5);
        candle.animate(fireAnimation);
    }

    public void setLEDS(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    public void setLEDSBlinking(int r, int g, int b) {
        StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, 0, 1.5, 68);
        candle.animate(strobeAnimation);
    }

    public void setAllianceColor() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            candle.setLEDs(255, 0, 0);
        } else {
            candle.setLEDs(0, 0, 255);
        }
    }
}