package frc.robot.SyncedLibraries;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.Controllers.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.DriveTrainBase;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

public class BasicFunctions {
  public static double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return (input - (Math.abs(input) / input * deadband)) / (1 - deadband);
    }
  }

  public static interface ControllerRunnable {
    public void run(ControllerBase controller);
  }

  /**
   * <b>ONLY FOR USE IN EMERGENCY</b>
   * <p>
   * YES, ACTUALLY
   * @param DriveTrain 
   */
  public static void KILLIT(DriveTrainBase DriveTrain) {
    DriverStation.reportError("KILLING IT", true);
    for (ManipulatorBase manipulator : ManipulatorBase.allManipulators) {
      DriverStation.reportWarning("ESTOP " + manipulator.getName(), false);
      manipulator.ESTOP();
      System.out.println("Done");
    }

    DriverStation.reportWarning("ESTOP DriveTrain", false);
    DriveTrain.ESTOP();
    System.out.println("Done");

    DriverStation.reportError("KILLED IT, EXITING NOW", false);
    System.exit(0);
  }
}
