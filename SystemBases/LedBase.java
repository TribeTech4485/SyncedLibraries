package frc.robot.SyncedLibraries.SystemBases;

import static edu.wpi.first.units.Units.Millimeters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedBase {
  protected final AddressableLED led;
  protected final AddressableLEDBuffer buffer;
  final Distance spacing;

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
   * @param length The length of the LED strip in LEDs.
   */
  public LedBase(int port, int length) {
    this(port, length, Millimeters.of(139 / 9));
  }

  public void periodic() {
    led.setData(buffer);
  }
}
