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
    return Math.pow(x, exponent) * Math.signum(x);
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
    // DriverStation.reportWarning("ESTOP DriveTrain", false);
    // DriveTrain.ESTOP();
    // System.out.println("Done");
    
    // DriverStation.reportError("KILLING IT", true);
    // for (Estoppable manipulator : Estoppable._getAllEstoppables()) {
      //   DriverStation.reportWarning("ESTOP " + manipulator.getName(), false);
    //   manipulator.ESTOP();
    //   System.out.println("Done");
    // }

    // DriverStation.reportError("KILLED IT, EXITING NOW", false);
    // System.exit(0);
  }

  /**
   * <b>ONLY FOR USE IN EMERGENCY</b>
   * <p>
   * YES, ACTUALLY
   * @deprecated Use "Estoppable.KILLIT()" instead
   */
  @Deprecated
  public static void KILLIT() {
    Estopable.KILLIT();
  //   DriverStation.reportError("KILLING IT", true);
  //   for (Estoppable manipulator : Estoppable._getAllEstoppables()) {
  //     DriverStation.reportWarning("ESTOP " + manipulator.getName(), false);
  //     manipulator.ESTOP();
  //     System.out.println("Done");
  //   }

  //   DriverStation.reportError("KILLED IT, EXITING NOW", false);
  //   System.exit(0);
  }
}
