package frc.robot.SyncedLibraries;

import frc.robot.SyncedLibraries.SystemBases.ControllerBase;

/** Where all of the controller objects are stored. */
public class Controllers {
  public static double joystickDeadband;
  public static double triggerDeadband;
  public static double triggerSetpoint;
  public static double joystickExponent;

  public Controllers(double joystickDeadband, double triggerDeadband, double triggerSetpoint, double joystickExponent) {
    Controllers.joystickDeadband = joystickDeadband;
    Controllers.triggerDeadband = triggerDeadband;
    Controllers.triggerSetpoint = triggerSetpoint;
    Controllers.joystickExponent = joystickExponent;
    fullUpdate();
  }

  public Controllers() {
    this(0.05, 0.05, 0.5, 1);
  }

  // Port-bound controllers
  public ControllerBase Zero;
  public ControllerBase One;
  public ControllerBase Two;
  public ControllerBase Three;
  public ControllerBase Four;
  public ControllerBase Five;
  public final ControllerBase ghostController = new ControllerBase(-1);

  /** Call this only upon inits */
  public void fullUpdate() {
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
}