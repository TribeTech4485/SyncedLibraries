package frc.robot.SyncedLibraries;

/**
 * Put in a public static variable, and update whenever doing somthing.
 * Now you can use this variable to check if the robot is driving, shooting,
 * etc.
 * <p>
 * KEEP UPDATED
 */
public class RobotState {
  public static enum RobotStateEnum {
    Driving, Shooting, Intaking, Climbing, Disabled
  }

  public static enum ManipulatorStateEnum {
    Intaking, Transporting, Held, Shooting, Empty
  }
}
