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
  double TargetX;
  double TargetY;
  double TargetArea;

  public LimelightBase() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    TargetX = tx.getDouble(0.0);
    TargetY = ty.getDouble(0.0);
    TargetArea = ta.getDouble(0.0);

    SmartDashboard.putNumber("TargetX", TargetX);
    SmartDashboard.putNumber("TargetY", TargetY);
    SmartDashboard.putNumber("TargetArea", TargetArea);
  }

  public double getX() {
    return TargetX;
  }

  public double getY() {
    return TargetY;
  }

  public double getArea() {
    return TargetArea;
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
  //Pipeline 1 = AprilTag Detection
  //Pipeline 2 = Retroreflective Detection
  public void setTarget(double target) {
    table.getEntry("pipeline").setNumber(target);
  }

  /*
   * Code to position the AprilTag to the center of the frame.
   */
    public void alignTag() {
    
    }
  }