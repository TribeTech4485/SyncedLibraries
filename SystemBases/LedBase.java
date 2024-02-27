package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

public class LedBase extends SubsystemBase {
  /** Creates a new LedBase. */
  public AddressableLED led;
  public LedSection[] sections;
  protected AddressableLEDBuffer mainBuffer;
  protected int counter = 0;

  public LedBase(int port, int activateTimer, int... sectionLengths) {
    int length = 0;
    for (int i = 0; i < sectionLengths.length; i++) {
      length += sectionLengths[i];
    }

    led = new AddressableLED(port);
    mainBuffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    led.setData(mainBuffer);
    this.sections = new LedSection[sectionLengths.length];
    for (int i = 0; i < sectionLengths.length; i++) {
      sections[i] = new LedSection(activateTimer, sectionLengths[i]);
    }
  }

  @Override
  public void periodic() {
    counter++;
    for (LedSection section : sections) {
      section.run();
    }
    combineSections();
    led.setData(mainBuffer);
  }

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

  private void combineSections() {
    AddressableLEDBuffer combinedBuffer = new AddressableLEDBuffer(mainBuffer.getLength());
    int i = 0;
    for (LedSection section : sections) {
      for (int j = 0; i < section.getBuffer().getLength(); j++) {
        combinedBuffer.setLED(i, section.getBuffer().getLED(j));
        i++;
      }
    }
    mainBuffer = combinedBuffer;
  }

  public class LedSection {
    AddressableLEDBuffer buffer;
    DisplayType displayType;
    int activateTimer;
    ArrayList<Color> colors;
    /** Section to follow if selected */
    LedSection followingSection = null;

    /**
     * Create like so:
     * <p>
     * new LedSection(0.1, Color.RED, Color.GREEN, new Color(0, 0, 255)).doCycle();
     * <p>
     * <p>
     * (0.1 second cycle between red, green, and blue)
     */
    public LedSection(int activateTimer, int length, Color... colors) {
      this.activateTimer = activateTimer;
      this.buffer = new AddressableLEDBuffer(length);
      this.colors = new ArrayList<Color>(colors.length);
      this.colors.addAll(List.of(colors));
    }

    public AddressableLEDBuffer getBuffer() {
      return buffer;
    }

    /**
     * Set the section to follow if used
     * <p>
     * <b>WATCH FOR INFINITE LOOPS</b>, calls recursively
     */
    public LedSection addFollowingSection(LedSection followingSection) {
      this.followingSection = followingSection;
      return this;
    }

    public LedSection addColor(Color... colors) {
      for (Color color : colors) {
        this.colors.add(color);
      }
      return this;
    }

    public void doRainbow() {
      this.displayType = DisplayType.RAINBOW;
    }

    public void doSolid() {
      this.displayType = DisplayType.SOLID;
    }

    public void doCycle() {
      this.displayType = DisplayType.CYCLE;
    }

    public void doAlternate() {
      this.displayType = DisplayType.ALTERNATE;
    }

    public void doMoveForward() {
      this.displayType = DisplayType.MOVE_FORWARD;
    }

    public void doMoveBackward() {
      this.displayType = DisplayType.MOVE_BACKWARD;
    }

    public void doOff() {
      this.displayType = DisplayType.OFF;
    }

    public void doSync() {
      if (followingSection == null) {
        new Throwable("Following section is null").printStackTrace();
        return;
      }
      this.displayType = DisplayType.SYNC;
    }

    public void doContinue() {
      if (followingSection == null) {
        new Throwable("Following section is null").printStackTrace();
        return;
      }
      this.displayType = DisplayType.CONTINUE;
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

    public void setActivateTimer(int activateTimer) {
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
      if (followingSection == null) {
        new Throwable("Following section is null").printStackTrace();
        return;
      }
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, followingSection.buffer.getLED(i));
      }
    }

    private void follow(int c) {
      if (followingSection == null) {
        new Throwable("Following section is null").printStackTrace();
        return;
      }
      run(followingSection.displayType, c + followingSection.buffer.getLength());
    }
  }
}
