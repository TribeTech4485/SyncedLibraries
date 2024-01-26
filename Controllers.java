// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
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
  }

  // Port-bound controllers
  public ControllerBase Zero;
  public ControllerBase One;
  public ControllerBase Two;
  public ControllerBase Three;
  public ControllerBase Four;
  public ControllerBase Five;

  // Only used when no controller is connected, made here to reduce waste memory
  public static ControllerBase ghostController = new ControllerBase(-1, true, false, false);

  // For automatically selecting controllers based on which one is being used
  // reccomended to be primary = driver, secondary = shooter
  public AutoControllerSelector primaryControllerSelector = new AutoControllerSelector(ghostController);
  public AutoControllerSelector secondaryControllerSelector = new AutoControllerSelector(ghostController);
  public AutoControllerSelector tertiaryControllerSelector = new AutoControllerSelector(ghostController);
  public ControllerBase Primary;
  public ControllerBase Secondary;
  public ControllerBase Tertiary;
  // idk why a third controller would be needed but it's here

  public void addControllers(AutoControllerSelector selector, ControllerBase... controllers) {
    selector.addController(controllers);
  }

  /** Add to robot periodic */
  public void updateAutoControllers() {
    Primary = primaryControllerSelector.getController();
    Secondary = secondaryControllerSelector.getController();
    Tertiary = tertiaryControllerSelector.getController();
  }

  /** Call this only upon inits */
  public void fullUpdate() {
    // I would imagine this to be a very expensive operation
    Zero = new ControllerBase(0);
    One = new ControllerBase(1);
    Two = new ControllerBase(2);
    Three = new ControllerBase(3);
    Four = new ControllerBase(4);
    Five = new ControllerBase(5);
    updateAutoControllers();
  }

  // TODO add joystick support
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

    public Trigger A;
    public Trigger B;
    public Trigger X;
    public Trigger Y;
    public Trigger LeftBumper;
    public Trigger RightBumper;
    public Trigger Share;
    public Trigger Options;
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

    /**
     * A class to handle controllers.
     * 
     * @param port The port of the controller. -1 for null controller.
     */
    public ControllerBase(int port) {
      this(port, new GenericHID(port).getType() == HIDType.kXInputGamepad,
          new GenericHID(port).getType() == HIDType.kXInputGamepad,
          new GenericHID(port).getType() == HIDType.kHIDJoystick);
    }

    public ControllerBase(int port, boolean isXbox, boolean isPS4, boolean isJoystick) {
      this.isXbox = isXbox;
      this.isPS4 = isPS4;
      this.isJoystick = isJoystick;
      this.port = port;

      // null controller
      if (port == -1) {
        this.A = new Trigger(() -> false);
        this.B = new Trigger(() -> false);
        this.X = new Trigger(() -> false);
        this.Y = new Trigger(() -> false);
        this.LeftBumper = new Trigger(() -> false);
        this.RightBumper = new Trigger(() -> false);
        this.Share = new Trigger(() -> false);
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
        return;
      }

      if (isXbox) {
        this.commObjectX = new CommandXboxController(port);
        this.objectX = commObjectX.getHID();
        this.A = commObjectX.a();
        this.B = commObjectX.b();
        this.X = commObjectX.x();
        this.Y = commObjectX.y();
        this.LeftBumper = commObjectX.leftBumper();
        this.RightBumper = commObjectX.rightBumper();
        this.Share = commObjectX.start();
        this.Options = commObjectX.back();
        this.LeftTrigger = commObjectX.leftTrigger();
        this.RightTrigger = commObjectX.rightTrigger();
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
      } else if (isPS4) {
        this.commObjectPS4 = new CommandPS4Controller(port);
        this.objectPS4 = commObjectPS4.getHID();
        this.A = commObjectPS4.cross();
        this.B = commObjectPS4.circle();
        this.X = commObjectPS4.square();
        this.Y = commObjectPS4.triangle();
        this.LeftBumper = commObjectPS4.L1();
        this.RightBumper = commObjectPS4.R1();
        this.Share = commObjectPS4.share();
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
        this.RightStickPress = commObjectPS4.L3();
      } else if (isJoystick) {
        this.commObjectJoystick = new CommandJoystick(port);
        this.objectJoystick = commObjectJoystick.getHID();
        this.A = new Trigger(() -> false);
        this.B = new Trigger(() -> false);
        this.X = new Trigger(() -> false);
        this.Y = new Trigger(() -> false);
        this.LeftBumper = new Trigger(() -> false);
        this.RightBumper = new Trigger(() -> false);
        this.Share = new Trigger(() -> false);
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
      } else {
        System.out.println("Controller on port " + port + " is not a valid controller.");
      }
    }

    public double getLeftXRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getLeftX();
      } else {
        return objectX.getLeftX();
      }
    }

    public double getLeftYRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getLeftY();
      } else {
        return objectX.getLeftY();
      }
    }

    public double getRightXRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getRightX();
      } else {
        return objectX.getRightX();
      }
    }

    public double getRightYRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getRightY();
      } else {
        return objectX.getRightY();
      }
    }

    public double getLeftTriggerRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getL2Axis();
      } else {
        return objectX.getLeftTriggerAxis();
      }
    }

    public double getRightTriggerRaw() {
      if (port == -1) {
        return 0;
      }
      if (isPS4) {
        return objectPS4.getR2Axis();
      } else {
        return objectX.getRightTriggerAxis();
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

    public boolean isBeingTouched() {
      if (A.getAsBoolean()) {
        return true;
      } else if (B.getAsBoolean()) {
        return true;
      } else if (X.getAsBoolean()) {
        return true;
      } else if (Y.getAsBoolean()) {
        return true;
      } else if (LeftBumper.getAsBoolean()) {
        return true;
      } else if (RightBumper.getAsBoolean()) {
        return true;
      } else if (Share.getAsBoolean()) {
        return true;
      } else if (Options.getAsBoolean()) {
        return true;
      } else if (LeftTrigger.getAsBoolean()) {
        return true;
      } else if (RightTrigger.getAsBoolean()) {
        return true;
      } else if (PovUp.getAsBoolean()) {
        return true;
      } else if (PovUpLeft.getAsBoolean()) {
        return true;
      } else if (PovUpRight.getAsBoolean()) {
        return true;
      } else if (PovDown.getAsBoolean()) {
        return true;
      } else if (PovDownLeft.getAsBoolean()) {
        return true;
      } else if (PovDownRight.getAsBoolean()) {
        return true;
      } else if (PovLeft.getAsBoolean()) {
        return true;
      } else if (PovRight.getAsBoolean()) {
        return true;
      } else if (LeftStickPress.getAsBoolean()) {
        return true;
      } else if (RightStickPress.getAsBoolean()) {
        return true;
      } else if (getLeftXRaw() != 0) {
        return true;
      } else if (getLeftYRaw() != 0) {
        return true;
      } else if (getRightXRaw() != 0) {
        return true;
      } else if (getRightYRaw() != 0) {
        return true;
      } else if (getLeftTriggerRaw() != 0) {
        return true;
      } else if (getRightTriggerRaw() != 0) {
        return true;
      } else {
        return false;
      }
    }

    public void setJoystickMultiplier(double multiplier) {
      this.joystickMultiplier = multiplier;
    }

    public void setTriggerMultiplier(double multiplier) {
      this.triggerMultiplier = multiplier;
    }

    public boolean isPluggedIn() {
      return objectX.isConnected() || objectPS4.isConnected() || objectJoystick.isConnected();
    }
  }
}