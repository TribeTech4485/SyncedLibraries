package frc.robot.SyncedLibraries;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Where all of the controller objects are stored. */
public class Controllers {
  private static double joystickDeadband;
  private static double triggerDeadband;

  public Controllers(double joystickDeadband, double triggerDeadband) {
    Controllers.joystickDeadband = joystickDeadband;
    Controllers.triggerDeadband = triggerDeadband;
    fullUpdate();
  }

  // Port-bound controllers
  public ControllerBase Zero;
  public ControllerBase One;
  public ControllerBase Two;
  public ControllerBase Three;
  public ControllerBase Four;
  public ControllerBase Five;

  /** Call this only upon inits */
  public void fullUpdate() {
    // I would imagine this to be a very expensive operation
    Zero = new ControllerBase(0);
    One = new ControllerBase(1);
    Two = new ControllerBase(2);
    Three = new ControllerBase(3);
    Four = new ControllerBase(4);
    Five = new ControllerBase(5);
  }

  public ControllerBase getPort(int port) {
    switch (port) {
      case 0:
        return Zero;
      case 1:
        return One;
      case 2:
        return Two;
      case 3:
        return Three;
      case 4:
        return Four;
      case 5:
        return Five;
      default:
        System.out.println("Port " + port + " is not a valid port. Returning ghost controller.");
        return null;
    }
  }

  /**
   * A class to contain all controller needs
   * If joystick, use left joystick for XY and objectJoystick for buttons
   */
  public static class ControllerBase {
    public boolean isPS4;
    public boolean isXbox;
    public boolean isJoystick = false;
    public int port;
    private double joystickMultiplier = 1;
    private double triggerMultiplier = 1;

    public CommandXboxController commObjectX;
    public XboxController objectX;
    public CommandPS4Controller commObjectPS4;
    public PS4Controller objectPS4;
    public CommandJoystick commObjectJoystick;
    public Joystick objectJoystick;

    public Supplier<Trigger> A;
    public Supplier<Trigger> B;
    public Supplier<Trigger> X;
    public Supplier<Trigger> Y;
    public Supplier<Trigger> LeftBumper;
    public Supplier<Trigger> RightBumper;
    public Supplier<Trigger> Share;
    public Supplier<Trigger> Options;
    public Supplier<Trigger> LeftTrigger;
    public Supplier<Trigger> RightTrigger;
    public Supplier<Trigger> PovUp;
    public Supplier<Trigger> PovUpLeft;
    public Supplier<Trigger> PovUpRight;
    public Supplier<Trigger> PovDown;
    public Supplier<Trigger> PovDownLeft;
    public Supplier<Trigger> PovDownRight;
    public Supplier<Trigger> PovLeft;
    public Supplier<Trigger> PovRight;
    public Supplier<Trigger> LeftStickPress;
    public Supplier<Trigger> RightStickPress;

    /**
     * A class to handle controllers.
     * Ghost controller is a controller that all axes and buttons return 0/false.
     * 
     * @param port The port of the controller. -1 for ghost controller.
     */
    public ControllerBase(int port) {
      // automatic controller type detection
      // this(port, new GenericHID(port).getType() == HIDType.kXInputGamepad,
      // new GenericHID(port).getType() == HIDType.kHIDGamepad,
      // new GenericHID(port).getType() == HIDType.kHIDJoystick);

      this(port, true, false, false);
    }

    public ControllerBase(int port, boolean isXbox, boolean isPS4, boolean isJoystick) {
      this.isXbox = isXbox;
      this.isPS4 = isPS4;
      this.isJoystick = isJoystick;
      this.port = port;

      if (port == -1) {
        System.out.println("Port " + port + " is a ghost controller.");
        initAsGhost();
      } else if (isXbox) {
        System.out.println("Controller on port " + port + " is an Xbox controller.");
        initAsXbox(port);
      } else if (isPS4) {
        System.out.println("Controller on port " + port + " is a PS4 controller.");
        initAsPS4(port);
      } else if (isJoystick) {
        System.out.println("Controller on port " + port + " is a joystick.");
        initAsJoystick(port);
      } else {
        System.out.println("Controller on port " + port
            + " is not a valid controller. Likely due to not being plugged in. Assuming Xbox.");
        initAsXbox(port);
        // initAsGhost();
      }
    }

    private void initAsJoystick(int port) {
      this.commObjectJoystick = new CommandJoystick(port);
      this.objectJoystick = commObjectJoystick.getHID();
      this.A = () -> new Trigger(() -> false);
      this.B = () -> new Trigger(() -> false);
      this.X = () -> new Trigger(() -> false);
      this.Y = () -> new Trigger(() -> false);
      this.LeftBumper = () -> new Trigger(() -> false);
      this.RightBumper = () -> new Trigger(() -> false);
      this.Share = () -> new Trigger(() -> false);
      this.Options = () -> new Trigger(() -> false);
      this.LeftTrigger = () -> new Trigger(() -> false);
      this.RightTrigger = () -> new Trigger(() -> false);
      this.PovUp = () -> new Trigger(() -> false);
      this.PovUpLeft = () -> new Trigger(() -> false);
      this.PovUpRight = () -> new Trigger(() -> false);
      this.PovDown = () -> new Trigger(() -> false);
      this.PovDownLeft = () -> new Trigger(() -> false);
      this.PovDownRight = () -> new Trigger(() -> false);
      this.PovLeft = () -> new Trigger(() -> false);
      this.PovRight = () -> new Trigger(() -> false);
      this.LeftStickPress = () -> new Trigger(() -> false);
      this.RightStickPress = () -> new Trigger(() -> false);
    }

    private void initAsPS4(int port) {
      this.commObjectPS4 = new CommandPS4Controller(port);
      this.objectPS4 = commObjectPS4.getHID();
      this.A = () -> commObjectPS4.cross();
      this.B = () -> commObjectPS4.circle();
      this.X = () -> commObjectPS4.square();
      this.Y = () -> commObjectPS4.triangle();
      this.LeftBumper = () -> commObjectPS4.L1();
      this.RightBumper = () -> commObjectPS4.R1();
      this.Share = () -> commObjectPS4.share();
      this.Options = () -> commObjectPS4.options();
      this.LeftTrigger = () -> commObjectPS4.L2();
      this.RightTrigger = () -> commObjectPS4.R2();
      this.PovUp = () -> commObjectPS4.povUp();
      this.PovUpLeft = () -> commObjectPS4.povUpLeft();
      this.PovUpRight = () -> commObjectPS4.povUpRight();
      this.PovDown = () -> commObjectPS4.povDown();
      this.PovDownLeft = () -> commObjectPS4.povDownLeft();
      this.PovDownRight = () -> commObjectPS4.povDownRight();
      this.PovLeft = () -> commObjectPS4.povLeft();
      this.PovRight = () -> commObjectPS4.povRight();
      this.LeftStickPress = () -> commObjectPS4.L3();
      this.RightStickPress = () -> commObjectPS4.L3();
    }

    private void initAsXbox(int port) {
      this.commObjectX = new CommandXboxController(port);
      this.objectX = commObjectX.getHID();
      this.A = () -> commObjectX.a();
      this.B = () -> commObjectX.b();
      this.X = () -> commObjectX.x();
      this.Y = () -> commObjectX.y();
      this.LeftBumper = () -> commObjectX.leftBumper();
      this.RightBumper = () -> commObjectX.rightBumper();
      this.Share = () -> commObjectX.start();
      this.Options = () -> commObjectX.back();
      this.LeftTrigger = () -> commObjectX.leftTrigger();
      this.RightTrigger = () -> commObjectX.rightTrigger();
      this.PovUp = () -> commObjectX.povUp();
      this.PovUpLeft = () -> commObjectX.povUpLeft();
      this.PovUpRight = () -> commObjectX.povUpRight();
      this.PovDown = () -> commObjectX.povDown();
      this.PovDownLeft = () -> commObjectX.povDownLeft();
      this.PovDownRight = () -> commObjectX.povDownRight();
      this.PovLeft = () -> commObjectX.povLeft();
      this.PovRight = () -> commObjectX.povRight();
      this.LeftStickPress = () -> commObjectX.leftStick();
      this.RightStickPress = () -> commObjectX.rightStick();
    }

    private void initAsGhost() {
      this.A = () -> new Trigger(() -> false);
      this.B = () -> new Trigger(() -> false);
      this.X = () -> new Trigger(() -> false);
      this.Y = () -> new Trigger(() -> false);
      this.LeftBumper = () -> new Trigger(() -> false);
      this.RightBumper = () -> new Trigger(() -> false);
      this.Share = () -> new Trigger(() -> false);
      this.Options = () -> new Trigger(() -> false);
      this.LeftTrigger = () -> new Trigger(() -> false);
      this.RightTrigger = () -> new Trigger(() -> false);
      this.PovUp = () -> new Trigger(() -> false);
      this.PovUpLeft = () -> new Trigger(() -> false);
      this.PovUpRight = () -> new Trigger(() -> false);
      this.PovDown = () -> new Trigger(() -> false);
      this.PovDownLeft = () -> new Trigger(() -> false);
      this.PovDownRight = () -> new Trigger(() -> false);
      this.PovLeft = () -> new Trigger(() -> false);
      this.PovRight = () -> new Trigger(() -> false);
      this.LeftStickPress = () -> new Trigger(() -> false);
      this.RightStickPress = () -> new Trigger(() -> false);
    }

    public double getLeftXRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getLeftX();
      } else if (isXbox) {
        return objectX.getLeftX();
      } else {
        return 0;
      }
    }

    public double getLeftYRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getLeftY();
      } else if (isXbox) {
        return objectX.getLeftY();
      } else {
        return 0;
      }
    }

    public double getRightXRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getRightX();
      } else if (isXbox) {
        return objectX.getRightX();
      } else {
        return 0;
      }
    }

    public double getRightYRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getRightY();
      } else if (isXbox) {
        return objectX.getRightY();
      } else {
        return 0;
      }
    }

    public double getLeftTriggerRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getL2Axis();
      } else if (isXbox) {
        return objectX.getLeftTriggerAxis();
      } else {
        return 0;
      }
    }

    public double getRightTriggerRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getR2Axis();
      } else if (isXbox) {
        return objectX.getRightTriggerAxis();
      } else {
        return 0;
      }
    }

    public double getLeftX() {
      return BasicFunctions.deadband(getLeftXRaw(), joystickDeadband) * joystickMultiplier;
    }

    public double getLeftY() {
      return BasicFunctions.deadband(getLeftYRaw(), joystickDeadband) * joystickMultiplier;
    }

    public double getRightX() {
      return BasicFunctions.deadband(getRightXRaw(), joystickDeadband) * joystickMultiplier;
    }

    public double getRightY() {
      return BasicFunctions.deadband(getRightYRaw(), joystickDeadband) * joystickMultiplier;
    }

    public double getLeftTrigger() {
      return BasicFunctions.deadband(getLeftTriggerRaw(), triggerDeadband) * triggerMultiplier;
    }

    public double getRightTrigger() {
      return BasicFunctions.deadband(getRightTriggerRaw(), triggerDeadband)
          * triggerMultiplier;
    }

    public void setRumble(RumbleType type, double rumble) {
      if (isPS4) {
        objectPS4.setRumble(type, rumble);
      } else {
        objectX.setRumble(type, rumble);
      }
    }

    /** ALWAYS RETURNS TRUE, DO NOT USE */
    @Deprecated
    public boolean isBeingTouched() {
      return true;
      // if (A.get().getAsBoolean()) {
      // return true;
      // } else if (B.get().getAsBoolean()) {
      // return true;
      // } else if (X.get().getAsBoolean()) {
      // return true;
      // } else if (Y.get().getAsBoolean()) {
      // return true;
      // } else if (LeftBumper.get().getAsBoolean()) {
      // return true;
      // } else if (RightBumper.get().getAsBoolean()) {
      // return true;
      // } else if (Share.get().getAsBoolean()) {
      // return true;
      // } else if (Options.get().getAsBoolean()) {
      // return true;
      // } else if (LeftTrigger.get().getAsBoolean()) {
      // return true;
      // } else if (RightTrigger.get().getAsBoolean()) {
      // return true;
      // } else if (PovUp.get().getAsBoolean()) {
      // return true;
      // } else if (PovUpLeft.get().getAsBoolean()) {
      // return true;
      // } else if (PovUpRight.get().getAsBoolean()) {
      // return true;
      // } else if (PovDown.get().getAsBoolean()) {
      // return true;
      // } else if (PovDownLeft.get().getAsBoolean()) {
      // return true;
      // } else if (PovDownRight.get().getAsBoolean()) {
      // return true;
      // } else if (PovLeft.get().getAsBoolean()) {
      // return true;
      // } else if (PovRight.get().getAsBoolean()) {
      // return true;
      // } else if (LeftStickPress.get().getAsBoolean()) {
      // return true;
      // } else if (RightStickPress.get().getAsBoolean()) {
      // return true;
      // } else if (getLeftXRaw() != 0) {
      // return true;
      // } else if (getLeftYRaw() != 0) {
      // return true;
      // } else if (getRightXRaw() != 0) {
      // return true;
      // } else if (getRightYRaw() != 0) {
      // return true;
      // } else if (getLeftTriggerRaw() != 0) {
      // return true;
      // } else if (getRightTriggerRaw() != 0) {
      // return true;
      // } else {
      // return false;
      // }
    }

    public boolean isJoysticksBeingTouched() {
      return getLeftX() != 0 || getLeftY() != 0 || getRightX() != 0 || getRightY() != 0;
    }

    public void setJoystickMultiplier(double multiplier) {
      this.joystickMultiplier = multiplier;
    }

    public void setTriggerMultiplier(double multiplier) {
      this.triggerMultiplier = multiplier;
    }

    public boolean isPluggedIn() {
      if (objectX != null) {
        return objectX.isConnected();
      } else if (objectPS4 != null) {
        return objectPS4.isConnected();
      } else if (objectJoystick != null) {
        return objectJoystick.isConnected();
      }
      return false;
    }
  }
}