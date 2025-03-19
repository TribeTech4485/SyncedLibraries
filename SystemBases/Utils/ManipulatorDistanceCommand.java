package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.SystemBases.PositionManipulatorBase;

import static edu.wpi.first.units.Units.Meters;

// TODO: move ff to here
public class ManipulatorDistanceCommand extends Command {
  protected PositionManipulatorBase manipulator;
  protected Distance position;
  protected Distance tolerance = Meters.of(0.1);
  protected PIDController pid;
  protected int onTargetCounterStart = 10;
  protected int onTargetCounter = onTargetCounterStart;
  protected boolean endOnTarget = false;
  public boolean atPosition = false;
  protected PIDConfig pidConfig;

  public ManipulatorDistanceCommand(PositionManipulatorBase manipulator, Distance position,
      PIDConfig pidConfig) {
    this.manipulator = manipulator;
    this.position = position;
    this.pidConfig = pidConfig;
    pid = new PIDController(0, 0, 0);
    synchronizePIDSettings();
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
    return Math.abs(manipulator.getPosition().minus(position).in(Meters)) < tolerance.in(Meters);
  }

  public void setEndOnTarget(boolean endOnTarget) {
    this.endOnTarget = endOnTarget;
  }

  protected void synchronizePIDSettings() {
    pidConfig.applyTo(pid);
  }
}
