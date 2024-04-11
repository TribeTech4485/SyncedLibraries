package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SyncedLibraries.AutoControllerSelector;
import frc.robot.SyncedLibraries.Controllers.ControllerBase;

public class TeleDriveCommandBase extends Command {
  protected ControllerBase[] controllers = null;
  protected AutoControllerSelector[] controllerSelectors = null;
  protected final boolean swerveDrive;
  protected double deadBand = 0.1;
  /** Set to false to disable warning upon startup */
  protected boolean defaultExecute = true;
  /** Up to 3 drivers */
  protected double[][] ys = new double[3][4];
  protected DriveTrainBase driveTrain;
  protected SwerveDriveBase swerveTrain;
  protected Subsystem actualDrivetrain;
  private boolean straightMode = false;

  /**
   * @deprecated Use AutoControllerSelectors instead
   */
  @Deprecated
  public TeleDriveCommandBase(DriveTrainBase driveTrain, boolean isSwerveDrive, ControllerBase... controller) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    swerveDrive = isSwerveDrive;
    this.controllers = controller;
  }

  public TeleDriveCommandBase(DriveTrainBase driveTrain,
      AutoControllerSelector... controllerSelector) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    swerveDrive = false;
    this.controllerSelectors = controllerSelector;
  }

  public TeleDriveCommandBase(SwerveDriveBase driveTrain,
      AutoControllerSelector... controllerSelector) {
    addRequirements(driveTrain);
    this.driveTrain = null;
    swerveDrive = true;
    this.controllerSelectors = controllerSelector;
  }

  @Override
  public void initialize() {
    actualDrivetrain = swerveDrive ? swerveTrain : driveTrain;
  }

  @Override
  public void execute() {
    ys = getJoys();
    if (swerveDrive) {
      // swerveTrain.update(false, ys[0][0], ys[0][2], ys[0][3]);
      double theta = Math.atan2(ys[0][0], ys[0][2]);
      swerveTrain.testSingleWheel(0, ys[0][1], theta);
      SmartDashboard.putNumber("Joystick X", ys[0][0]);
      SmartDashboard.putNumber("Joystick Y", ys[0][1]);
      SmartDashboard.putNumber("Joystick Rotate", ys[0][2]);
    } else {
      SmartDashboard.putNumber("Left Y", ys[0][0]);
      SmartDashboard.putNumber("Right Y", ys[0][1]);
        if (Math.abs(ys[0][0] - ys[0][1]) <= 0.05 || straightMode) {
          double newY = (ys[0][0] + ys[0][1]) / 2;
          ys[0][0] = newY;
          ys[0][1] = newY;
        }
        driveTrain.doTankDrive(ys[0][0], ys[0][1]);
    }

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
    driveTrain.stop();
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isDisabled();
  }

  protected double[][] getJoys() {
    if (controllerSelectors != null) {
      double[][] joys = new double[3][4];
      for (int i = 0; i < controllerSelectors.length; i++) {
        if (controllerSelectors[i] == null) {
          joys[i] = zeros();
          continue;
        }
        joys[i][0] = controllerSelectors[i].getLeftY();
        joys[i][1] = controllerSelectors[i].getRightY();
        joys[i][2] = controllerSelectors[i].getLeftX();
        joys[i][3] = controllerSelectors[i].getRightX();
      }
      return joys;
    }
    if (controllers != null) {
      for (ControllerBase controller : controllers) {
        if (controller != null) {
          // if (controller.isJoysticksBeingTouched()) {
          return new double[][] { {
              -controller.getLeftY(),
              -controller.getRightY(),
              -controller.getLeftX(),
              -controller.getRightX()
          },
              zeros(),
              zeros()
          };
          // }
        }
      }
    }
    return new double[][] { zeros(), zeros(), zeros() };
  }

  private double[] zeros() {
    return new double[] { 0, 0, 0, 0 };
  }
}
