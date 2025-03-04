package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.SystemBases.AngleManipulatorBase;
import static edu.wpi.first.units.Units.Radians;

// TODO: PURGE THIS FILE
public class ManipulatorAngleCommand extends Command {
  protected AngleManipulatorBase manipulator;
  protected Angle targetPosition;
  protected PIDController pid;
  protected int onTargetCounterStart = 10;
  protected int onTargetCounter = onTargetCounterStart;
  public Angle tolerance = Radians.of(0.1);
  protected boolean endOnTarget = false;
  public boolean atPosition = false;
  protected final PIDConfig pidConfig;

  public ManipulatorAngleCommand(AngleManipulatorBase manipulator, Angle position,
      PIDConfig pidConfig) {
    this.manipulator = manipulator;
    this.targetPosition = position;
    this.pidConfig = pidConfig;
    pid = new PIDController(0, 0, 0);
    synchronizePIDSettings();
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
    synchronizePIDSettings();
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

  public boolean isAtPosition() {
    if (currentlyAtPosition()) {
      onTargetCounter--;
    } else {
      onTargetCounter = onTargetCounterStart;
    }
    return onTargetCounter <= 0;
  }

  private boolean currentlyAtPosition() {
    return manipulator.getAngle().minus(targetPosition).abs(Radians) < tolerance.in(Radians);
  }

  public void setEndOnTarget(boolean endOnTarget) {
    this.endOnTarget = endOnTarget;
  }

  protected void synchronizePIDSettings() {
    pidConfig.applyTo(pid);
  }
}
