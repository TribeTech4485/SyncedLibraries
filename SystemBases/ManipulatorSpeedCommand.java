package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;

public class ManipulatorSpeedCommand extends Command {
  private ManipulatorBase manipulator;
  private double targetSpeed;
  private double tolerance;
  private PIDController pid;

  public ManipulatorSpeedCommand(ManipulatorBase manipulator, double speed, double tolerance, double kP, double kI, double kD) {
    this.manipulator = manipulator;
    this.targetSpeed = speed;
    this.tolerance = tolerance;
    pid = new PIDController(kP, kI, kD);
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.stop();
    pid.reset();
    pid.setSetpoint(targetSpeed);
    if (targetSpeed == Integer.MAX_VALUE) {
      DriverStation.reportError("ManipulatorSpeedCommand: speed not set", true);
      CommandScheduler.getInstance().cancel(this);
    }
  }

  @Override
  public void execute() {
    manipulator.setPower(pid.calculate(manipulator.getCurrentSpeed(), targetSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    manipulator.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(manipulator.getCurrentSpeed() - targetSpeed) < tolerance;
  }

  public void setTargetSpeed(double speed) {
    this.targetSpeed = speed;
  }

  public double getTargetSpeed() {
    return targetSpeed;
  }
}
