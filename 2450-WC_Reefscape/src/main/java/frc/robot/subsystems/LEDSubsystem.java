package frc.robot.subsystems;

import java.util.NoSuchElementException;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private CANdle candle = new CANdle(9);

    public LEDSubsystem() {
    }

    @Override
    public void periodic() {
    }

    public CANdle getCANdle() {
        return candle;
    }

    public void fireLEDS() {
        FireAnimation fireAnimation = new FireAnimation(1, 0.4, 308, 0.5, 0.5);
        candle.animate(fireAnimation);
    }

    public void setLEDColor(int r, int g, int b, int w) {
        candle.clearAnimation(0);
        candle.setLEDs(r, g, b, w, 0, 308);
        // candle.configBrightnessScalar(1);
        // StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, w, 0.0, 308);
        // candle.animate(strobeAnimation);
    }

    public void setLEDSBlinking(int r, int g, int b, int w) {
        candle.configBrightnessScalar(1);

        StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, w, 0.08, 308);
        candle.animate(strobeAnimation);
    }

    public void setLEDSFlowing(int r, int g, int b, int w) {
        candle.configBrightnessScalar(1);

        ColorFlowAnimation colorFlowAnimation = new ColorFlowAnimation(r, g, b, w, 2, 308, Direction.Forward);
        candle.animate(colorFlowAnimation);
    }

    public void blinkAllianceColor() {
        candle.clearAnimation(0);
        StrobeAnimation strobeAnimation;
        try {
            if (DriverStation.getAlliance().orElseThrow() == Alliance.Red) {
                strobeAnimation = new StrobeAnimation(255, 0, 0, 0, 0.08, 308);
            } else {
                strobeAnimation = new StrobeAnimation(0, 0, 255, 0, 0.08, 308);
            }
        } catch (NoSuchElementException e) {
            strobeAnimation = new StrobeAnimation(0, 0, 255, 0, 0.08, 308);
        }

        candle.animate(strobeAnimation);
    }

    public void setAllianceColor() {
        candle.clearAnimation(0);
        try {
            if (DriverStation.getAlliance().orElseThrow() == Alliance.Red) {
                candle.setLEDs(255, 0, 0);
            } else {
                candle.setLEDs(0, 0, 255);
            }
        } catch (NoSuchElementException e) {
            candle.setLEDs(0, 0, 255);
        }
    }
}