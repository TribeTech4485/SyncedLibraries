// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A subsystem, but you have to have an ESTOP function */
public abstract class Estoppable extends SubsystemBase {
  /**
   * <b>EMERGENCY STOP</b>
   * <p>
   * WILL STOP THE MANIPULATOR
   * <p>
   * Recomended to set brake mode to true, unless if the manipulator could be
   * in a position that would be dangerous or would trap a piece.
   */
  public abstract void ESTOP();
}
