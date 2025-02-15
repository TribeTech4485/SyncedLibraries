package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.SystemBases.AngleManipulatorBase;
import static edu.wpi.first.units.Units.Radians;

public class ManipulatorAngleCommand extends Command {
  protected AngleManipulatorBase manipulator;
  protected Angle targetPosition;
  protected Angle tolerance;
  protected PIDController pid;
  protected int onTargetCounterStart = 10;
  protected int onTargetCounter = onTargetCounterStart;
  protected boolean endOnTarget = false;
  public boolean atPosition = false;

  public ManipulatorAngleCommand(AngleManipulatorBase manipulator, Angle position, Angle tolerance,
      double kP, double kI, double kD) {
    this.manipulator = manipulator;
    this.targetPosition = position;
    this.tolerance = tolerance;
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(tolerance.in(Radians));
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.stop();
    pid.reset();
    pid.setSetpoint(targetPosition.in(Radians));
  }

  @Override
  public void execute() {
    manipulator.setPower(pid.calculate(manipulator.getAngle().in(Radians), targetPosition.in(Radians)),
        false);
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

  public void setTargetPosition(Angle position) {
    this.targetPosition = position;
  }

  public Angle getTargetPosition() {
    return targetPosition;
  }

  public Angle getTolerance() {
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
    return Math.abs(manipulator.getAngle().minus(targetPosition).in(Radians)) < tolerance.in(Radians) * 1.5;
  }

  public void setEndOnTarget(boolean endOnTarget) {
    this.endOnTarget = endOnTarget;
  }
}
