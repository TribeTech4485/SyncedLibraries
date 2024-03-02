package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * Example creation:
 * 
 * <pre>
 * <code>
 * LED = new LedBase(0, 30, 30);
 * LED.sections[0].init(1, 1, Color.kRed, Color.kGreen, Color.kBlue).doCycle();
 * LED.sections[1].init(1, 1, Color.kRed, Color.kGreen, Color.kBlue).doMoveForward();
 * </code>
 * </pre>
 */
public class LedBase extends SubsystemBase {
  public AddressableLED led;
  public LedSection[] sections;
  protected AddressableLEDBuffer mainBuffer;
  protected int counter = 0;

  /** Be sure to run sections[i].init() */
  public LedBase(int port, int... sectionLengths) {
    int length = 0;
    for (int i = 0; i < sectionLengths.length; i++) {
      length += sectionLengths[i];
    }

    led = new AddressableLED(port);
    mainBuffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    for (int i = 0; i < length; i++) {
      mainBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(mainBuffer);
    led.start();
    this.sections = new LedSection[sectionLengths.length];
    for (int i = 0; i < sectionLengths.length; i++) {
      sections[i] = new LedSection(sectionLengths[i]);
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
    /** Moving rainbow forward */
    RAINBOW_FORWARD,
    /** Moving rainbow backward */
    RAINBOW_BACKWARD,
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
      for (int j = 0; j < section.getBuffer().getLength(); j++, i++) {
        combinedBuffer.setLED(i, section.getBuffer().getLED(j));
      }
    }
    // System.out.println(i + " Total lights" + mainBuffer.getLength());
    // System.out.println(sections[0].getBuffer().getLength() +
    // sections[1].getBuffer().getLength());
    mainBuffer = combinedBuffer;
  }

  public class LedSection {
    AddressableLEDBuffer buffer;
    DisplayType displayType;
    double speed;
    double width;
    ArrayList<Color> colors;
    /** Section to follow if selected */
    LedSection followingSection = null;

    /*
     * public LedSection(int speed, int width, int length, Color... colors) {
     * this.speed = speed;
     * this.width = width;
     * this.buffer = new AddressableLEDBuffer(length);
     * this.colors = new ArrayList<Color>(colors.length);
     * this.colors.addAll(List.of(colors));
     * }
     */

    /** Creates a blank section, only for temporary use */
    public LedSection(int length) {
      this.speed = 1;
      this.width = 1;
      this.buffer = new AddressableLEDBuffer(length);
      this.colors = new ArrayList<Color>();
      this.colors.add(new Color(0, 0, 0));
    }

    /**
     * After this, run one of the do methods
     * 
     * @param speed  Ticks per color change
     * @param width  Width of the color before next color
     * @param length Length of the section
     * @param colors Colors to cycle through
     */
    public LedSection init(double speed, double width, Color... colors) {
      this.speed = speed;
      this.width = width;
      this.colors = new ArrayList<Color>(colors.length);
      this.colors.addAll(List.of(colors));
      return this;
    }

    /** Set the section to continue/sync with if selected by the do method */
    public LedSection setFollower(LedSection followingSection) {
      this.followingSection = followingSection;
      return this;
    }

    private AddressableLEDBuffer getBuffer() {
      return buffer;
    }

    /** Moving rainbow forward */
    public void doRainbow() {
      this.displayType = DisplayType.RAINBOW_FORWARD;
    }

    /** Moving rainbow backward */
    public void doRainbowBackward() {
      this.displayType = DisplayType.RAINBOW_BACKWARD;
    }

    /** Rolid color, first only */
    public void doSolid() {
      this.displayType = DisplayType.SOLID;
    }

    /** Rlinking between colors */
    public void doCycle() {
      this.displayType = DisplayType.CYCLE;
    }

    /** Rotate through colors list (staticly) */
    public void doAlternate() {
      this.displayType = DisplayType.ALTERNATE;
    }

    /** Rotate through colors list (moving) */
    public void doMoveForward() {
      this.displayType = DisplayType.MOVE_FORWARD;
    }

    /** Rotate through colors list (moving backwards) */
    public void doMoveBackward() {
      this.displayType = DisplayType.MOVE_BACKWARD;
    }

    /** No comment */
    public void doOff() {
      this.displayType = DisplayType.OFF;
    }

    /**
     * Sync with {@link #setFollower(LedSection)} section
     * <p>
     * <b>WATCH FOR INFINITE LOOPS</b>, following calls recursively
     */
    public void doSync(LedSection followingSection) {
      this.followingSection = followingSection;
      this.displayType = DisplayType.SYNC;
    }

    /**
     * Continue from {@link #setFollower(LedSection)} section
     * <p>
     * <b>WATCH FOR INFINITE LOOPS</b>, following calls recursively
     */
    public void doContinue(LedSection followingSection) {
      this.followingSection = followingSection;
      this.displayType = DisplayType.CONTINUE;
    }

    private void run() {
      run(displayType, counter);
    }

    private void run(DisplayType dType, int count) {
      switch (dType) {
        case RAINBOW_FORWARD:
          rainbow(count);
          break;
        case RAINBOW_BACKWARD:
          rainbow(-count);
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

    private void rainbow(int c) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setHSV(i, (int) (((c * speed) + (i * 180 / width / buffer.getLength())) % (180)), 255, 255);
      }
    }

    private void solid(Color color) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    }

    private void cycle(int c) {
      int number = (int) (c * speed) % colors.size();
      solid(colors.get(number));
    }

    private void alternate() {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, colors.get((int) ((i / width) % colors.size())));
      }
    }

    private void moveForward(int c) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, colors.get((int) (((i / width) + (c / speed)) % colors.size())));
      }
    }

    private void moveBackward(int c) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, colors.get((int) (Math.abs((-i / width) + (c / speed)) % colors.size())));
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
