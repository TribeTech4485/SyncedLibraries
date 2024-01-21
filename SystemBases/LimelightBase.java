// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightBase extends SubsystemBase {
  /** Creates a new LimelightBase. */
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  // read values periodically
  double x;
  double y;
  double area;

  public LimelightBase() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getArea() {
    return area;
  }

  public void turnOnLED() {
    table.getEntry("ledMode").setNumber(3);
  }

  public void turnOffLED() {
    table.getEntry("ledMode").setNumber(1);
  }

  public void setCamModeVision() {
    table.getEntry("camMode").setNumber(0);
  }

  public void setCamModeDriver() {
    table.getEntry("camMode").setNumber(1);
  }

  /** Sets the pipeline to use for the limelight */
  public void setTarget(double target) {
    table.getEntry("pipeline").setNumber(target);
  }
}
