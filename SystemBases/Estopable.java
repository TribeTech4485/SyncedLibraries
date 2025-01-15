// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A subsystem, but you have to have an ESTOP function */
public abstract class Estopable extends SubsystemBase {
  private static LinkedList<Estopable> allEstoppables = new LinkedList<Estopable>();

  /**
   * Silently adds itself to the list of all estoppables for in case of emergency
   */
  public Estopable() {
    allEstoppables.add(this);
    System.out.println("Creating subsystem " + getName());
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

    DriverStation.reportError("KILLED IT, EXITING NOW", false);
    System.exit(0);
  }
}
