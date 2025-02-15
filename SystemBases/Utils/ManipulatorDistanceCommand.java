package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.SystemBases.PositionManipulatorBase;

import static edu.wpi.first.units.Units.Meters;

public class ManipulatorDistanceCommand extends Command {
  protected PositionManipulatorBase manipulator;
  protected Distance position;
  protected Distance tolerance;
  protected PIDController pid;
  protected int onTargetCounterStart = 10;
  protected int onTargetCounter = onTargetCounterStart;
  protected boolean endOnTarget = false;
  public boolean atPosition = false;

  public ManipulatorDistanceCommand(PositionManipulatorBase manipulator, Distance position, Distance tolerance,
      double kP, double kI, double kD) {
    this.manipulator = manipulator;
    this.position = position;
    this.tolerance = tolerance;
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(tolerance.in(Meters));
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.stop();
    pid.reset();
    pid.setSetpoint(position.in(Meters));
  }

  @Override
  public void execute() {
    manipulator.setPower(pid.calculate(manipulator.getPosition().in(Meters), position.in(Meters)), false);
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

  public void setTargetPosition(Distance position) {
    this.position = position;
  }

  public Distance getTargetPosition() {
    return position;
  }

  public Distance getTolerance() {
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
    return Math.abs(manipulator.getPosition().minus(position).in(Meters)) < tolerance.in(Meters) * 1.5;
  }

  public void setEndOnTarget(boolean endOnTarget) {
    this.endOnTarget = endOnTarget;
  }
}
