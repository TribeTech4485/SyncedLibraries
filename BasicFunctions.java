package frc.robot.SyncedLibraries;

import frc.robot.SyncedLibraries.SystemBases.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.Estopable;

public class BasicFunctions {
  public static double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return (input - (Math.abs(input) / input * deadband)) / (1 - deadband);
    }
  }

  public static double smartExp(double x, double exponent) {
    double result = Math.pow(x, exponent);
    if (Math.signum(result) == Math.signum(x)) {
      return result;
    }
    return -result;
  }

  public static interface ControllerRunnable {
    public void run(ControllerBase controller);
  }

  /**
   * <b>ONLY FOR USE IN EMERGENCY</b>
   * <p>
   * YES, ACTUALLY
   * 
   * @param DriveTrain
   * @deprecated Use "Estoppable.KILLIT()" instead
   */
  @Deprecated
  public static void KILLIT(Estopable DriveTrain) {
    Estopable.KILLIT();
  }

  /**
   * <b>ONLY FOR USE IN EMERGENCY</b>
   * <p>
   * YES, ACTUALLY
   * 
   * @deprecated Use "Estoppable.KILLIT()" instead
   */
  @Deprecated
  public static void KILLIT() {
    Estopable.KILLIT();
  }
}
