package frc.robot.SyncedLibraries.SystemBases;

import static edu.wpi.first.units.Units.Millimeters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LedBase extends SubsystemBase {
  protected final AddressableLED led;
  protected final AddressableLEDBuffer buffer;
  protected final Distance spacing;

  /**
   * @param port       The PWM port number to which the LED strip is connected.
   * @param length     The number of LEDs in the strip.
   * @param ledSpacing The distance between each LED in the strip.
   */
  public LedBase(int port, int length, Distance ledSpacing) {
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    led.setData(buffer);
    led.start();
    spacing = ledSpacing;
  }

  /**
   * Constructs a new LedBase object with a default spacing of 15.44mm.
   *
   * @param port   The PWM port number to which the LED strip is connected.
   * @param length The length of the LED strip in # of LEDs.
   */
  public LedBase(int port, int length) {
    this(port, length, Millimeters.of(139 / 9));
  }

  /** led.setData(buffer) */
  @Override
  public void periodic() {
    led.setData(buffer);
    applyPatterns();
  }

  protected abstract void applyPatterns();

  /**
   * If not set, returns random color.
   * 
   * @return The color of the alliance, fails to blue if no data is available.
   */
  protected Color getAllianceColor() {
    try {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return Color.kBlue;
      } else {
        return Color.kRed;
      }
    } catch (Exception e) {
      return Color.kYellow;
    }
  }
}
