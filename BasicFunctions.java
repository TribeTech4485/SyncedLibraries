package frc.robot.SyncedLibraries;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.SystemBases.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.DriveTrainBase;
import frc.robot.SyncedLibraries.SystemBases.Estoppable;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

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
   * @deprecated Use {@link #KILLIT()} instead
   */
  @Deprecated
  public static void KILLIT(Estoppable DriveTrain) {

    DriverStation.reportWarning("ESTOP DriveTrain", false);
    DriveTrain.ESTOP();
    System.out.println("Done");

    DriverStation.reportError("KILLING IT", true);
    for (Estoppable manipulator : ManipulatorBase.allManipulators) {
      DriverStation.reportWarning("ESTOP " + manipulator.getName(), false);
      manipulator.ESTOP();
      System.out.println("Done");
    }

    DriverStation.reportError("KILLED IT, EXITING NOW", false);
    System.exit(0);
  }

  /**
   * <b>ONLY FOR USE IN EMERGENCY</b>
   * <p>
   * YES, ACTUALLY
   */
  public static void KILLIT() {
    DriverStation.reportError("KILLING IT", true);
    for (Estoppable manipulator : ManipulatorBase.allManipulators) {
      DriverStation.reportWarning("ESTOP " + manipulator.getName(), false);
      manipulator.ESTOP();
      System.out.println("Done");
    }

    DriverStation.reportError("KILLED IT, EXITING NOW", false);
    System.exit(0);
  }
}
