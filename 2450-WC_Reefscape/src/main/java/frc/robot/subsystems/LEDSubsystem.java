package frc.robot.subsystems;

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
        candle.configBrightnessScalar(1);
        FireAnimation fireAnimation = new FireAnimation(1, 0.4, 68, 0.5, 0.5);
        candle.animate(fireAnimation);
    }

    public void setLEDS(int r, int g, int b, int w) {
        candle.configBrightnessScalar(1);
        StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, w, 0.0, 308);
        candle.animate(strobeAnimation);
    }

    public void setLEDSBlinking(int r, int g, int b, int w) {
        candle.configBrightnessScalar(1);

        StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, w, 0.03, 308);
        candle.animate(strobeAnimation);
    }

    public void setLEDSFlowing(int r, int g, int b) {
        candle.configBrightnessScalar(1);

        ColorFlowAnimation colorFlowAnimation = new ColorFlowAnimation(r, g, b, 0, 0.5, 308, Direction.Forward);
        candle.animate(colorFlowAnimation);
    }

    

    public void setAllianceColor() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            candle.setLEDs(255, 0, 0);
        } else {
            candle.setLEDs(0, 0, 255);
        }
    }
}