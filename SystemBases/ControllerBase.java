package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SyncedLibraries.BasicFunctions;
import frc.robot.SyncedLibraries.Controllers;

/**
 * A class to contain all controller needs
 * If joystick, use left joystick for XY and the buttons list for buttons
 */
public class ControllerBase {
  public final boolean isPS4;
  public final boolean isXbox;
  public final boolean isJoystick;
  public final int port;

  public CommandXboxController commObjectX;
  public XboxController objectX;
  public CommandPS4Controller commObjectPS4;
  public PS4Controller objectPS4;
  public CommandJoystick commObjectJoystick;
  public Joystick objectJoystick;

  public Trigger A;
  public Trigger B;
  public Trigger X;
  public Trigger Y;
  public Trigger LeftBumper;
  public Trigger RightBumper;
  public Trigger Start;
  public Trigger Options; // The left one on Xbox
  public Trigger LeftTrigger;
  public Trigger RightTrigger;
  public Trigger PovUp;
  public Trigger PovUpLeft;
  public Trigger PovUpRight;
  public Trigger PovDown;
  public Trigger PovDownLeft;
  public Trigger PovDownRight;
  public Trigger PovLeft;
  public Trigger PovRight;
  public Trigger LeftStickPress;
  public Trigger RightStickPress;
  public Trigger ESTOPCondition;
  public Trigger AnyButton;
  public Trigger RightJoyMoved;
  public Trigger LeftJoyMoved;
  /**
   * <b>IS ONE INDEXED TO MATCH WITH {@link #getRawButton()}, 0th returns false
   */
  public Trigger[] buttons;

  /**
   * A class to handle controllers.
   * Ghost controller is a controller that all axes and buttons return 0/false.
   * <p>
   * <b>Currently this only creates an xbox controller. Use
   * {@link #ControllerBase(int, boolean, boolean, boolean)} for more options.</b>
   * 
   * @param port The port of the controller. -1 for ghost controller.
   */
  public ControllerBase(int port) {
    // TODO: automatic controller type detection
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
    ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));
  }

  private void initAsJoystick(int port) {
    this.commObjectJoystick = new CommandJoystick(port);
    this.objectJoystick = commObjectJoystick.getHID();
    this.A = commObjectJoystick.button(1);
    this.B = commObjectJoystick.button(2);
    this.X = commObjectJoystick.button(3);
    this.Y = commObjectJoystick.button(4);
    this.LeftBumper = commObjectJoystick.button(5);
    this.RightBumper = commObjectJoystick.button(6);
    this.Start = commObjectJoystick.button(7);
    this.Options = commObjectJoystick.button(8);
    this.LeftTrigger = commObjectJoystick.button(9);
    this.RightTrigger = commObjectJoystick.button(10);
    this.PovUp = commObjectJoystick.button(11);
    this.PovUpLeft = commObjectJoystick.button(12);
    this.PovUpRight = commObjectJoystick.button(13);
    this.PovDown = commObjectJoystick.button(14);
    this.PovDownLeft = commObjectJoystick.button(15);
    this.PovDownRight = commObjectJoystick.button(16);
    this.PovLeft = commObjectJoystick.button(17);
    this.PovRight = commObjectJoystick.button(18);
    this.LeftStickPress = commObjectJoystick.button(19);
    this.RightStickPress = commObjectJoystick.button(20);
    this.RightJoyMoved = new Trigger(() -> (getLeftX() != 0 || getLeftY() != 0));
    this.LeftJoyMoved = new Trigger(() -> (getLeftX() != 0 || getLeftY() != 0));

    // int buttonCount = objectJoystick.getButtonCount();
    buttons = new Trigger[13]; // TODO: set to "buttonCount + 1"
    for (int i = 0; i < buttons.length; i++) {
      if (i == 0) {
        buttons[i] = new Trigger(() -> false);
        continue;
      }
      final int j = i;
      buttons[i] = commObjectJoystick.button(j);
    }

    // All the buttons on the base section
    this.ESTOPCondition = commObjectJoystick.button(3)
        .and(commObjectJoystick.button(4))
        .and(commObjectJoystick.button(5))
        .and(commObjectJoystick.button(6));

    this.AnyButton = A.or(B).or(X).or(Y).or(LeftBumper)
        .or(RightBumper).or(Start).or(Options).or(LeftTrigger)
        .or(RightTrigger).or(PovUp).or(PovUpLeft).or(PovUpRight)
        .or(PovDown).or(PovDownLeft).or(PovDownRight).or(PovLeft)
        .or(PovRight).or(LeftStickPress).or(RightStickPress);
  }

  private void initAsPS4(int port) {
    this.commObjectPS4 = new CommandPS4Controller(port);
    this.objectPS4 = commObjectPS4.getHID();
    this.A = commObjectPS4.cross();
    this.B = commObjectPS4.circle();
    this.X = commObjectPS4.square();
    this.Y = commObjectPS4.triangle();
    this.LeftBumper = commObjectPS4.L1();
    this.RightBumper = commObjectPS4.R1();
    this.Start = commObjectPS4.share();
    this.Options = commObjectPS4.options();
    this.LeftTrigger = commObjectPS4.L2();
    this.RightTrigger = commObjectPS4.R2();
    this.PovUp = commObjectPS4.povUp();
    this.PovUpLeft = commObjectPS4.povUpLeft();
    this.PovUpRight = commObjectPS4.povUpRight();
    this.PovDown = commObjectPS4.povDown();
    this.PovDownLeft = commObjectPS4.povDownLeft();
    this.PovDownRight = commObjectPS4.povDownRight();
    this.PovLeft = commObjectPS4.povLeft();
    this.PovRight = commObjectPS4.povRight();
    this.LeftStickPress = commObjectPS4.L3();
    this.RightStickPress = commObjectPS4.R3();
    this.RightJoyMoved = new Trigger(() -> (getRightX() != 0 || getRightY() != 0));
    this.LeftJoyMoved = new Trigger(() -> (getLeftX() != 0 || getLeftY() != 0));

    int buttonCount = objectPS4.getButtonCount();
    buttons = new Trigger[buttonCount + 1];
    for (int i = 0; i < buttonCount + 1; i++) {
      if (i == 0) {
        buttons[i] = new Trigger(() -> false);
        continue;
      }
      final int j = i;
      buttons[i] = commObjectPS4.button(j);
    }

    // All the buttons your hands are already resting on
    this.ESTOPCondition = commObjectPS4.L3()
        .and(commObjectPS4.R3())
        .and(commObjectPS4.L1())
        .and(commObjectPS4.R1())
        .and(commObjectPS4.L2())
        .and(commObjectPS4.R2());

    this.AnyButton = A.or(B).or(X).or(Y).or(LeftBumper)
        .or(RightBumper).or(Start).or(Options).or(LeftTrigger)
        .or(RightTrigger).or(PovUp).or(PovUpLeft).or(PovUpRight)
        .or(PovDown).or(PovDownLeft).or(PovDownRight).or(PovLeft)
        .or(PovRight).or(LeftStickPress).or(RightStickPress);
  }

  private void initAsXbox(int port) {
    this.commObjectX = new CommandXboxController(port);
    this.objectX = commObjectX.getHID();
    this.A = commObjectX.a();
    this.B = commObjectX.b();
    this.X = commObjectX.x();
    this.Y = commObjectX.y();
    this.LeftBumper = commObjectX.leftBumper();
    this.RightBumper = commObjectX.rightBumper();
    this.Start = commObjectX.start();
    this.Options = commObjectX.back();
    this.LeftTrigger = commObjectX.leftTrigger(Controllers.triggerSetpoint);
    this.RightTrigger = commObjectX.rightTrigger(Controllers.triggerSetpoint);
    this.PovUp = commObjectX.povUp();
    this.PovUpLeft = commObjectX.povUpLeft();
    this.PovUpRight = commObjectX.povUpRight();
    this.PovDown = commObjectX.povDown();
    this.PovDownLeft = commObjectX.povDownLeft();
    this.PovDownRight = commObjectX.povDownRight();
    this.PovLeft = commObjectX.povLeft();
    this.PovRight = commObjectX.povRight();
    this.LeftStickPress = commObjectX.leftStick();
    this.RightStickPress = commObjectX.rightStick();
    this.RightJoyMoved = new Trigger(() -> (getRightX() != 0 || getRightY() != 0));
    this.LeftJoyMoved = new Trigger(() -> (getLeftX() != 0 || getLeftY() != 0));

    int buttonCount = objectX.getButtonCount();
    buttons = new Trigger[buttonCount + 1];
    for (int i = 0; i < buttonCount + 1; i++) {
      if (i == 0) {
        buttons[i] = new Trigger(() -> false);
        continue;
      }
      final int j = i;
      buttons[i] = commObjectX.button(j);
    }

    // All the buttons your hands are already resting on
    this.ESTOPCondition = commObjectX.leftStick()
        .and(commObjectX.rightStick())
        .and(commObjectX.leftBumper())
        .and(commObjectX.rightBumper())
        .and(commObjectX.leftTrigger())
        .and(commObjectX.rightTrigger());

    this.AnyButton = A.or(B).or(X).or(Y).or(LeftBumper)
        .or(RightBumper).or(Start).or(Options).or(LeftTrigger)
        .or(RightTrigger).or(PovUp).or(PovUpLeft).or(PovUpRight)
        .or(PovDown).or(PovDownLeft).or(PovDownRight).or(PovLeft)
        .or(PovRight).or(LeftStickPress).or(RightStickPress);
  }

  private void initAsGhost() {
    this.A = new Trigger(() -> false);
    this.B = new Trigger(() -> false);
    this.X = new Trigger(() -> false);
    this.Y = new Trigger(() -> false);
    this.LeftBumper = new Trigger(() -> false);
    this.RightBumper = new Trigger(() -> false);
    this.Start = new Trigger(() -> false);
    this.Options = new Trigger(() -> false);
    this.LeftTrigger = new Trigger(() -> false);
    this.RightTrigger = new Trigger(() -> false);
    this.PovUp = new Trigger(() -> false);
    this.PovUpLeft = new Trigger(() -> false);
    this.PovUpRight = new Trigger(() -> false);
    this.PovDown = new Trigger(() -> false);
    this.PovDownLeft = new Trigger(() -> false);
    this.PovDownRight = new Trigger(() -> false);
    this.PovLeft = new Trigger(() -> false);
    this.PovRight = new Trigger(() -> false);
    this.LeftStickPress = new Trigger(() -> false);
    this.RightStickPress = new Trigger(() -> false);
    this.ESTOPCondition = new Trigger(() -> false);

    buttons = new Trigger[20];
    for (int i = 0; i < 20; i++) {
      buttons[i] = new Trigger(() -> false);
    }

  }

  /** If joystick: X-axis */
  public double getLeftXRaw() {
    if (port == -1) {
      return 0;
    }
    if (isPS4) {
      return objectPS4.getLeftX();
    } else if (isXbox) {
      return objectX.getLeftX();
    } else if (isJoystick) {
      return objectJoystick.getX();
    } else {
      return 0;
    }
  }

  /** If joystick: Y-axis */
  public double getLeftYRaw() {
    if (port == -1) {
      return 0;
    }
    if (isPS4) {
      return objectPS4.getLeftY();
    } else if (isXbox) {
      return objectX.getLeftY();
    } else if (isJoystick) {
      return objectJoystick.getY();
    } else {
      return 0;
    }
  }

  /** If joystick: Twist-axis */
  public double getRightXRaw() {
    if (port == -1) {
      return 0;
    }
    if (isPS4) {
      return objectPS4.getRightX();
    } else if (isXbox) {
      return objectX.getRightX();
    } else if (isJoystick) {
      return objectJoystick.getZ();
    } else {
      return 0;
    }
  }

  /** If joystick: Throttle-axis */
  public double getRightYRaw() {
    if (port == -1) {
      return 0;
    }
    if (isPS4) {
      return objectPS4.getRightY();
    } else if (isXbox) {
      return objectX.getRightY();
    } else if (isJoystick) {
      return objectJoystick.getThrottle();
    } else {
      return 0;
    }
  }

  /** If joystick: 1 or 0 if trigger pressed */
  public double getLeftTriggerRaw() {
    if (port == -1) {
      return 0;
    }
    if (isPS4) {
      return objectPS4.getL2Axis();
    } else if (isXbox) {
      return objectX.getLeftTriggerAxis();
    } else if (isJoystick) {
      return objectJoystick.getTrigger() ? 1 : 0;
    } else {
      return 0;
    }
  }

  /** If joystick: 1 or 0 if trigger pressed */
  public double getRightTriggerRaw() {
    if (port == -1) {
      return 0;
    }
    if (isPS4) {
      return objectPS4.getR2Axis();
    } else if (isXbox) {
      return objectX.getRightTriggerAxis();
    } else if (isJoystick) {
      return objectJoystick.getTrigger() ? 1 : 0;
    } else {
      return 0;
    }
  }

  /** If joystick: X-axis */
  public double getLeftX() {
    return BasicFunctions.smartExp(
        BasicFunctions.deadband(getLeftXRaw(), Controllers.joystickDeadband), Controllers.joystickExponent);
  }

  /** If joystick: Y-axis */
  public double getLeftY() {
    return BasicFunctions.smartExp(
        BasicFunctions.deadband(getLeftYRaw(), Controllers.joystickDeadband), Controllers.joystickExponent);
  }

  /** If joystick: Twist-axis */
  public double getRightX() {
    return BasicFunctions.smartExp(
        BasicFunctions.deadband(getRightXRaw(), Controllers.joystickDeadband), Controllers.joystickExponent);
  }

  /** If joystick: Throttle-axis */
  public double getRightY() {
    return BasicFunctions.smartExp(
        BasicFunctions.deadband(getRightYRaw(), Controllers.joystickDeadband), Controllers.joystickExponent);
  }

  /** If joystick: 1 or 0 if trigger pressed */
  public double getLeftTrigger() {
    return BasicFunctions.deadband(getLeftTriggerRaw(), Controllers.triggerDeadband);
  }

  /** If joystick: 1 or 0 if trigger pressed */
  public double getRightTrigger() {
    return BasicFunctions.deadband(getRightTriggerRaw(), Controllers.triggerDeadband);
  }

  public void setRumble(RumbleType type, double rumble) {
    if (isPS4) {
      objectPS4.setRumble(type, rumble);
    } else if (isXbox) {
      objectX.setRumble(type, rumble);
    } else if (isJoystick) {
      // How would this work? ;)
      objectJoystick.setRumble(type, rumble);
    }
  }

  public boolean areJoysticksBeingTouched() {
    if (!isJoystick) {
      return getLeftX() != 0 || getLeftY() != 0 || getRightX() != 0 || getRightY() != 0;
    } else {
      return getLeftX() != 0 || getLeftY() != 0 || getRightX() != 0;
    }
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

  public boolean getRawButton(int button) {
    if (isPS4) {
      return objectPS4.getRawButton(button);
    } else if (isXbox) {
      return objectX.getRawButton(button);
    } else if (isJoystick) {
      return objectJoystick.getRawButton(button);
    } else {
      return false;
    }
  }

  /** AKA the D-PAD, returns int degrees where 0 is up */
  public int getPOV() {
    if (isPS4) {
      return objectPS4.getPOV();
    } else if (isXbox) {
      return objectX.getPOV();
    } else if (isJoystick) {
      return objectJoystick.getPOV();
    } else {
      return -1;
    }
  }
}