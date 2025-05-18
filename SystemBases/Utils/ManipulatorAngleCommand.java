package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.SystemBases.AngleManipulatorBase;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

// TODO: move ff to here

public class ManipulatorAngleCommand extends Command {
  protected AngleManipulatorBase manipulator;
  protected Angle targetPosition;
  protected ProfiledPIDController pid;
  protected int onTargetCounterStart = 10;
  protected int onTargetCounter = onTargetCounterStart;
  protected boolean endOnTarget = false;
  public boolean atPosition = false;
  protected final PIDConfig pidConfig;

  public ManipulatorAngleCommand(AngleManipulatorBase manipulator, Angle position,
      PIDConfig pidConfig) {
    this.manipulator = manipulator;
    this.targetPosition = position;
    this.pidConfig = pidConfig;
    pid = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    pid.enableContinuousInput(0 * Math.PI, 2 * Math.PI);
    synchronizePIDSettings();
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.stop();
    resetController();
    pid.setGoal(targetPosition.in(Radians));
  }

  @Override
  public void execute() {
    synchronizePIDSettings();
    pid.setGoal(targetPosition.in(Radians) % (2 * Math.PI));
    manipulator.setVoltage(
        Volts.of(pid.calculate(manipulator.getAngle().in(Radians) % (2 * Math.PI)) + getFF()),
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
    return endOnTarget && atPosition && false;
  }

  private double getFF() {
    switch (pidConfig.feedForwardType) {
      case SimpleMotor:
        if (pidConfig.A == 0.0) {
          return pidConfig.S * Math.signum(pid.getSetpoint().velocity) + pidConfig.V * pid
              .getSetpoint().velocity;
        } else {
          double A = -pidConfig.V / pidConfig.A;
          double B = 1.0 / pidConfig.A;
          double A_d = Math.exp(A * 0.02);
          double B_d = 1.0 / A * (A_d - 1.0) * B;
          return pidConfig.S * Math.signum(manipulator.getCurrentSpeed().in(RadiansPerSecond))
              + 1.0 / B_d * (pid.getSetpoint().velocity - A_d
                  * manipulator.getCurrentSpeed().in(RadiansPerSecond));
        }

      case Elevator:
        if (pidConfig.A == 0.0) {
          return pidConfig.S * Math.signum(pid.getSetpoint().velocity) + pidConfig.G
              + pidConfig.V * pid.getSetpoint().velocity;
        } else {
          double A = -pidConfig.V / pidConfig.A;
          double B = 1.0 / pidConfig.A;
          double A_d = Math.exp(A * 0.02);
          double B_d = 1.0 / A * (A_d - 1.0) * B;
          return pidConfig.G
              + pidConfig.S * Math.signum(manipulator.getCurrentSpeed().in(RadiansPerSecond))
              + 1.0 / B_d
                  * (pid.getSetpoint().velocity - A_d * manipulator.getCurrentSpeed().in(RadiansPerSecond));
        }

      case Arm:
        if (pidConfig.A == 0.0) {
          return pidConfig.S * Math.signum(pid.getSetpoint().velocity) +
              pidConfig.G * Math.cos(manipulator.getAngle().in(Radians)) +
              pidConfig.V * pid.getSetpoint().velocity;
        } else {
          double A = -pidConfig.V / pidConfig.A;
          double B = 1.0 / pidConfig.A;
          double A_d = Math.exp(A * 0.02);
          double B_d = 1.0 / A * (A_d - 1.0) * B;
          return pidConfig.G * Math.cos(manipulator.getAngle().in(Radians))
              + pidConfig.S * Math.signum(manipulator.getCurrentSpeed().in(RadiansPerSecond))
              + 1.0 / B_d
                  * (pid.getSetpoint().velocity - A_d * manipulator.getCurrentSpeed().in(RadiansPerSecond));
        }
      default:
        throw new IllegalArgumentException("FeedForward controller not set");
    }
  }

  public void setTargetPosition(Angle position) {
    this.targetPosition = position;
  }

  public Angle getTargetPosition() {
    return targetPosition;
  }

  /** In terms of Radians and Radians/sec */
  public TrapezoidProfile.State getSetpoint() {
    return pid.getSetpoint();
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
    return manipulator.getAngle().minus(targetPosition).abs(Radians) < pidConfig.angleTolerance.in(Radians);
  }

  public void setEndOnTarget(boolean endOnTarget) {
    this.endOnTarget = endOnTarget;
  }

  protected void synchronizePIDSettings() {
    pidConfig.applyTo(pid, RadiansPerSecond);
  }

  public void resetController() {
    pid.reset(manipulator.getAngle().in(Radians),
        manipulator.getCurrentSpeed().in(RadiansPerSecond));
  }
}
