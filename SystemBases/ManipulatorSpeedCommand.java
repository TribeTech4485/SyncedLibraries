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
  private int onTargetCounterStart = 10;
  private int onTargetCounter = onTargetCounterStart;

  public ManipulatorSpeedCommand(ManipulatorBase manipulator, double speed, double tolerance, double kP, double kI,
      double kD) {
    this.manipulator = manipulator;
    this.targetSpeed = speed;
    this.tolerance = tolerance;
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(tolerance);
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
    double pidSpeed = pid.calculate(manipulator.getCurrentSpeed(), targetSpeed);
    manipulator.setPower(pidSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {
    manipulator.stop();
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(manipulator.getCurrentSpeed() - targetSpeed) < tolerance * 1.5) {
      onTargetCounter--;
    } else {
      onTargetCounter = onTargetCounterStart;
    }
    return onTargetCounter <= 0;
  }

  public void setTargetSpeed(double speed) {
    this.targetSpeed = speed;
    initialize();
  }

  public double getTargetSpeed() {
    return targetSpeed;
  }

  public double getTolerance() {
    return tolerance;
  }

  public boolean isAtSpeed() {
    if (currentlyAtSpeed()) {
      onTargetCounter--;
    } else {
      onTargetCounter = onTargetCounterStart;
    }
    return onTargetCounter <= 0;
  }

  private boolean currentlyAtSpeed() {
    return Math.abs(manipulator.getCurrentSpeed() - targetSpeed) < tolerance;
  }
}
