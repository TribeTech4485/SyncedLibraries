package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

public class TeleDriveCommandBase extends Command {
  // TODO: Add swerve drive support
  protected ControllerBase[] controllers = null;
  protected final boolean swerveDrive;
  protected double deadBand = 0.1;
  /** Set to false to disable warning upon startup */
  protected boolean defaultExecute = true;
  /**
   * Up to 3 drivers
   * <p>
   * First axis is controller number, typically just use 0
   * <p>
   * Second axis is the joystick values, in the order of:
   * <p>
   * <i>
   * Left Y, Right Y, Left X, Right X, POV
   * </i>
   */
  protected double[][] ys = new double[2][5];
  protected SwerveDriveBase swerveTrain; // TODO
  protected boolean straightMode = false;

  public TeleDriveCommandBase(SwerveDriveBase driveTrain, ControllerBase controller) {
    addRequirements(driveTrain);
    swerveDrive = true;
    this.swerveTrain = driveTrain;
  }

  @Override
  public void execute() {
    ys = getJoys();
    swerveTrain.inputDrivingX_Y_A(ys[0][2], ys[0][0], Math.atan2(ys[0][1], ys[0][3]), (int) ys[0][4]);

    // This is a warning to the programmer that they should override this method
    if (defaultExecute) {
      defaultExecute = false;
      DriverStation.reportWarning("TeleDriveCommandBase: execute() not overridden.\n" +
          "Nothing is wrong with this, but reccomended to put further joystick controls here.", false);
    }
  }

  public void straightDrive(boolean straight) {
    straightMode = straight;
  }

  @Override
  public void end(boolean interrupted) {
    // driveTrain.stop();
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isDisabled();
  }

  /**
   * The first array is the driver/priority, nested array is the joystick values
   * <br>
   * Left Y, Right Y, Left X, Right X, POV
   * If joystick: X, Y, Twist, Throttle, POV
   */
  protected double[][] getJoys() {
    if (controllers != null) {
      double[][] toReturn = new double[3][5];
      for (int i = 0; i < controllers.length; i++) {
        if (controllers[i] != null) {
          toReturn[i] = new double[] {
              controllers[i].getLeftY(),
              controllers[i].getRightY(),
              controllers[i].getLeftX(),
              controllers[i].getRightX(),
              controllers[i].getPOV()
          };
        } else {
          toReturn[i] = zeros();
        }
      }
      return toReturn;
    }
    return new double[][] { zeros(), zeros(), zeros() };
  }

  private double[] zeros() {
    return new double[] { 0, 0, 0, 0, 0 };
  }
}
