package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SyncedLibraries.Controllers.ControllerBase;

public class TeleDriveCommandBase extends Command {
  // TODO: Add swerve drive support
  // TODO: Add multi-controller support
  ControllerBase[] controllers;
  boolean swerveDrive = false;

  public TeleDriveCommandBase(ControllerBase... controller) {
    addRequirements(Robot.DriveTrain);
    this.controllers = controller;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // TODO Add swerve drive support
    double[] ys = getJoys();
    if (swerveDrive) {
    } else {
      Robot.DriveTrain.doTankDrive(ys[0], ys[1]);
    }
    Robot.Turret.setPower(ys[2], false);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.DriveTrain.stop();
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isDisabled();
  }

  private double[] getJoys() {
    for (ControllerBase controller : controllers) {
      if (controller.isBeingTouched()) {
        return new double[] {
            controller.getLeftY(),
            controller.getRightY(),
            controller.getLeftX(),
            controller.getRightX()
        };
      }
    }
    return new double[] { 0, 0, 0, 0 };
  }
}
