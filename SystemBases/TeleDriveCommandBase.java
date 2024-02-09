package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SyncedLibraries.Controllers.ControllerBase;

public class TeleDriveCommandBase extends Command {
  // TODO: Add swerve drive support
  // TODO: Add multi-controller support
  ControllerBase[] controllers;
  boolean swerveDrive = false;
  double deadBand = 0.1;

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
    double[] ys = procDeadBand(getJoys());
    SmartDashboard.putNumber("Left Y", -ys[0]);
    SmartDashboard.putNumber("Right Y", -ys[1]);
    if (swerveDrive) {
    } else {
      Robot.DriveTrain.doTankDrive(ys[0], ys[1]);
    }
    // Robot.Turret.setPower(ys[2], false);
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

  private double[] procDeadBand(double[] inputs) {
    for (double input : inputs) {
      if (Math.abs(input) < deadBand) {
        input = 0;
      } else {
        input = (input - (Math.abs(input) / (input / deadBand))) / (1 - deadBand);
      }
    }
    return inputs;
  }
}
