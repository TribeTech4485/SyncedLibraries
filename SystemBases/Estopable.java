// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases;

import java.util.LinkedList;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A {@link SubsystemBase}, but you have to have an ESTOP function */
public abstract class Estopable extends SubsystemBase {
  private static LinkedList<Estopable> allEstoppables = new LinkedList<Estopable>();
  private static Command disableCommand;
  private static boolean previouslyDisabled = true;
  private static boolean allowFullEstop = !DriverStation.isFMSAttached();

  /**
   * Silently adds itself to the list of all estoppables for in case of emergency
   */
  public Estopable() {
    allEstoppables.add(this);
    System.out.println("Creating subsystem " + getName());

    if (disableCommand == null) {
      // injects the function onDisableAll that runs when the robot is disabled
      // TODO: verify that onDisable runs when the robot is disabled
      disableCommand = new Command() {
        @Override
        public void execute() {
          boolean isDisabled = DriverStation.isDisabled();
          if (isDisabled && !previouslyDisabled) {
            onDisableAll();
          }
          previouslyDisabled = isDisabled;
        }

        @Override
        public boolean isFinished() {
          return false;
        }

        @Override
        public boolean runsWhenDisabled() {
          return true;
        }
      };
      disableCommand.schedule();
    }
  }

  /**
   * <b>EMERGENCY STOP</b>
   * <p>
   * WILL STOP THE SUBSYSTEM
   * <p>
   * Recomended to set brake mode to true, unless if the system could be
   * in a position that would be dangerous or would trap a piece.
   */
  public abstract void ESTOP();

  /**
   * <b>ONLY FOR USE IN CASE OF AN <i>EMERGENCY
   * <p>
   * NO TOUCHIE</i></b>
   */
  public static void KILLIT() {
    DriverStation.reportError("KILLING IT", true);
    for (Estopable stopable : allEstoppables) {
      DriverStation.reportWarning("ESTOPing " + stopable.getName(), false);
      stopable.ESTOP();
      System.out.println("Done");
    }

    if (allowFullEstop) {
      DriverStation.reportError("KILLED IT, EXITING NOW", false);
      System.exit(0);
    } else {
      DriverStation.reportError("CONNECTED TO FMS, NOT KILLING THE CODE", false);
    }
  }

  /**
   * <b>NOT FOR INSTANTIATED OBJECTS</b>
   * <p>
   * FOR {@link #KILLIT()}, don't stop the code.
   */
  public static void dontAllowFullEstop() {
    allowFullEstop = false;
  }

  public static Estopable[] getAllEstopables() {
    return allEstoppables.toArray(new Estopable[0]);
  }

  /**
   * Runs when robot is disabled
   * <p>
   * Reccomended to stop motors and save motor configurations
   */
  public void onDisable() {
    // Override this if you want to do something when the robot is disabled
  }

  private static void onDisableAll() {
    for (Estopable stopable : allEstoppables) {
      stopable.onDisable();
    }
    System.out
        .println("=======================This is a test to see if onDisable works (in Estopable)=====================");
  }

}
