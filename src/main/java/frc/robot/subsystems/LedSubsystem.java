package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple wrapper around a CTRE CANdle to control a short LED strip.
 */
public class LedSubsystem extends SubsystemBase {
  private final CANdle candle;
  private final int ledCount;

  public LedSubsystem(int canId, int ledCount) {
    this.ledCount = ledCount;
    candle = new CANdle(canId);
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.statusLedOffWhenActive = true;
    cfg.disableWhenLOS = false;
    candle.configAllSettings(cfg);
    setOff();
  }

  /** Set all LEDs solid green. */
  public void setGreen() {
    candle.setLEDs(0, 255, 0, 0, 0, ledCount);
  }

  /** Turn all LEDs off. */
  public void setOff() {
    candle.setLEDs(0, 0, 0, 0, 0, ledCount);
  }
}
