package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SyncedLibraries.Controllers.ControllerBase;

public class TeleDriveCommandBase extends Command {
  protected ControllerBase[] controllers;
  protected boolean swerveDrive = false;
  protected double deadBand = 0.1;
  protected boolean defaultExecute = true;
  protected double[] ys = new double[4];

  public TeleDriveCommandBase(DriveTrainBase driveTrain, boolean isSwerveDrive, ControllerBase... controller) {
    addRequirements(driveTrain);
    swerveDrive = isSwerveDrive;
    this.controllers = controller;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ys = getJoys();
    SmartDashboard.putNumber("Left Y", ys[0]);
    SmartDashboard.putNumber("Right Y", ys[1]);
    if (swerveDrive) {
      // TODO: Add swerve drive support
    } else {
      Robot.DriveTrain.doTankDrive(ys[0], ys[1]);
    }

    // This is a warning to the programmer that they should override this method
    if (defaultExecute) {
      defaultExecute = false;
      DriverStation.reportWarning("TeleDriveCommandBase: execute() not overridden.\n" +
          "Nothing is wrong with this, but reccomended to put further joystick controls here.", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.DriveTrain.stop();
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isDisabled();
  }

  protected double[] getJoys() {
    for (ControllerBase controller : controllers) {
      if (controller.isJoysticksBeingTouched()) {
        return new double[] {
            -controller.getLeftY(),
            -controller.getRightY(),
            -controller.getLeftX(),
            -controller.getRightX()
        };
      }
    }
    return new double[] { 0, 0, 0, 0 };
  }
}
