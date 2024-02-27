package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

public class LedBase extends SubsystemBase {
  /** Creates a new LedBase. */
  public AddressableLED led;
  AddressableLEDBuffer mainBuffer;
  int numSections;
  ArrayList<LedSection> sections = new ArrayList<LedSection>(numSections);
  int counter = 0;

  
  /**
   * Represents a base class for controlling an addressable LED strip.
   *
   * @param port      The port number of the addressable LED strip.
   * @param length    The length of the addressable LED strip.
   * @param sections  The number of sections in the addressable LED strip.
   */
  public LedBase(int port, int length, int sections) {
    led = new AddressableLED(port);
    mainBuffer = new AddressableLEDBuffer(length);
    led.setLength(mainBuffer.getLength());
    led.setData(mainBuffer);
    this.numSections = sections;
  }

  @Override
  public void periodic() {
    counter++;
    AddressableLEDBuffer[] buffers = new AddressableLEDBuffer[sections.size()];
    for (int i = 0; i < sections.size(); i++) {
      sections.get(i).run();
      buffers[i] = sections.get(i).getBuffer();
    }
    mainBuffer = combineSections(sections);
    led.setData(mainBuffer);
  }

  /** SYNC/CONTINUE ONLY WORKS ON SECOND SECTION */
  enum DisplayType {
    /** Moving rainbow */
    RAINBOW,
    /** Rolid color, first only */
    SOLID,
    /** Rlinking between colors */
    CYCLE,
    /** Rotate through colors list (staticly) */
    ALTERNATE,
    /** Rotate through colors list (moving) */
    MOVE_FORWARD,
    /** Rotate through colors list (moving) */
    MOVE_BACKWARD,
    /** No comment */
    OFF,
    /** Sync with following section */
    SYNC,
    /**
     * Continue from following section
     * <p>
     * <b>WATCH FOR INFINITE LOOPS</b>, calls recursively
     */
    CONTINUE
  }

  private AddressableLEDBuffer combineSections(ArrayList<LedSection> sections) {
    LedSection[] newSections = new LedSection[sections.size()];
    for (var i = 0; i < sections.size(); i++) {
      newSections[i] = sections.get(i);
    }
    AddressableLEDBuffer combinedBuffer = new AddressableLEDBuffer(mainBuffer.getLength());
    int i = 0;
    for (LedSection section : newSections) {
      for (int j = 0; i < section.getBuffer().getLength(); j++) {
        combinedBuffer.setLED(i, section.getBuffer().getLED(j));
        i++;
      }
    }
    return combinedBuffer;
  }

  class LedSection {
    AddressableLEDBuffer buffer;
    DisplayType displayType;
    double activateTimer;
    ArrayList<Color> colors;
    /** Section to follow if selected */
    LedSection followingSection;

    public LedSection(DisplayType displayType, double activateTimer, ArrayList<Color> colors,
        LedSection followingSection) {
      this.displayType = displayType;
      this.activateTimer = activateTimer;
      this.colors = colors;
      this.followingSection = followingSection;
    }

    public AddressableLEDBuffer getBuffer() {
      return buffer;
    }

    public void run() {
      run(displayType, counter);
    }

    private void run(DisplayType dType, int count) {
      switch (dType) {
        case RAINBOW:
          rainbow(count);
          break;
        case SOLID:
          solid(colors.get(0));
          break;
        case CYCLE:
          cycle(count);
          break;
        case ALTERNATE:
          alternate();
          break;
        case MOVE_FORWARD:
          moveForward(count);
          break;
        case MOVE_BACKWARD:
          moveBackward(count);
          break;
        case OFF:
          solid(new Color(0, 0, 0));
          break;
        case SYNC:
          sync();
          break;
        case CONTINUE:
          follow(count % buffer.getLength());
          break;
      }
    }

    public void setType(DisplayType displayType) {
      this.displayType = displayType;
    }

    public void setActivateTimer(double activateTimer) {
      this.activateTimer = activateTimer;
    }

    private void rainbow(int c) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setHSV(i, (int) (c * activateTimer) % 180, 255, 255);
      }
    }

    private void solid(Color color) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    }

    private void cycle(int c) {
      if (c % activateTimer == 0) {
        int number = (c % (int) activateTimer) % colors.size();
        solid(colors.get(number));
      }
    }

    private void alternate() {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, colors.get(i % colors.size()));
      }
    }

    private void moveForward(int c) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, colors.get((i + c) % colors.size()));
      }
    }

    private void moveBackward(int c) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, colors.get((i - c) % colors.size()));
      }
    }

    private void sync() {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, followingSection.buffer.getLED(i));
      }
    }

    private void follow(int c) {
      run(followingSection.displayType, c + followingSection.buffer.getLength());
    }
  }
}
