package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;

public class ManipulatorMoveCommand extends Command {
  private ManipulatorBase manipulator;
  private double position;
  private double tolerance;
  private PIDController pid;
  private int onTargetCounterStart = 10;
  private int onTargetCounter = onTargetCounterStart;
  private boolean endOnTarget = false;
  private boolean atPosition = false;

  public ManipulatorMoveCommand(ManipulatorBase manipulator, double position, double tolerance, double kP, double kI, double kD) {
    this.manipulator = manipulator;
    this.position = position;
    this.tolerance = tolerance;
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(tolerance);
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.stop();
    pid.reset();
    pid.setSetpoint(position);
    if (position == Integer.MAX_VALUE) {
      // https://www.youtube.com/watch?v=dQw4w9WgXcQ //
      DriverStation.reportError("ManipulatorMoveCommand: position not set", true);
      CommandScheduler.getInstance().cancel(this);
    }
  }

  @Override
  public void execute() {
    manipulator.setPower(pid.calculate(manipulator.getPosition(), position), false);
    atPosition = isAtPosition();
  }

  @Override
  public void end(boolean interrupted) {
    manipulator.stop();
    if (interrupted) {
      DriverStation.reportWarning("ManipulatorMoveCommand: interrupted", false);
    }
  }

  @Override
  public boolean isFinished() {
    return endOnTarget && atPosition;
  }

  public ManipulatorMoveCommand setTargetPosition(double position) {
    this.position = position;
    return this;
  }

  public double getTargetPosition() {
    return position;
  }

  public double getTolerance() {
    return tolerance;
  }

  public boolean isAtPosition() {
    if (currentlyAtPosition()) {
      onTargetCounter--;
    } else {
      onTargetCounter = onTargetCounterStart;
    }
    return onTargetCounter <= 0;
  }

  private boolean currentlyAtPosition() {
    return Math.abs(manipulator.getPosition() - position) < tolerance * 1.5;
  }

  public ManipulatorMoveCommand setEndOnTarget(boolean endOnTarget) {
    this.endOnTarget = endOnTarget;
    return this;
  }
}
