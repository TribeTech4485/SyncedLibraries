package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.SystemBases.PositionManipulatorBase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

// TODO: move ff to here
public class ManipulatorDistanceCommand extends Command {
  protected PositionManipulatorBase manipulator;
  protected Distance position;
  protected ProfiledPIDController pid;
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
    pid = new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(0, 0));
    synchronizePIDSettings();
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.stop();
    pid.reset(manipulator.getPosition().in(Meters), manipulator.getVelocity().in(MetersPerSecond));
    pid.setGoal(position.in(Meters));
  }

  @Override
  public void execute() {
    synchronizePIDSettings();

    pid.setGoal(position.in(Meters));
    manipulator.setVoltage(Volts.of(pid.calculate(manipulator.getPosition().in(Meters)) + getFF()),
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

  /** Get the feed-forward's produced power */
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
          return pidConfig.S * Math.signum(manipulator.getVelocity().in(MetersPerSecond))
              + 1.0 / B_d * (pid.getSetpoint().velocity - A_d
                  * manipulator.getVelocity().in(MetersPerSecond));
        }

      case Elevator:
        if (pidConfig.A == 0.0) {
          // no acceleration is easier to compute
          return pidConfig.S * Math.signum(pid.getSetpoint().velocity) + pidConfig.G
              + pidConfig.V * pid.getSetpoint().velocity;
        } else {
          double A = -pidConfig.V / pidConfig.A;
          double B = 1.0 / pidConfig.A;
          double A_d = Math.exp(A * 0.02);
          double B_d = 1.0 / A * (A_d - 1.0) * B;
          return pidConfig.G
              + pidConfig.S * Math.signum(manipulator.getVelocity().in(MetersPerSecond))
              + 1.0 / B_d
                  * (pid.getSetpoint().velocity - A_d * manipulator.getVelocity().in(MetersPerSecond));
        }

      case Arm:
        if (pidConfig.A == 0.0) {
          // no acceleration is easier to compute
          return pidConfig.S * Math.signum(pid.getSetpoint().velocity) +
              pidConfig.G * Math.cos(manipulator.getPosition().in(Meters)) +
              pidConfig.V * pid.getSetpoint().velocity;
        } else {
          double A = -pidConfig.V / pidConfig.A;
          double B = 1.0 / pidConfig.A;
          double A_d = Math.exp(A * 0.02);
          double B_d = 1.0 / A * (A_d - 1.0) * B;
          return pidConfig.G * Math.cos(manipulator.getPosition().in(Meters))
              + pidConfig.S * Math.signum(manipulator.getVelocity().in(MetersPerSecond))
              + 1.0 / B_d
                  * (pid.getSetpoint().velocity - A_d * manipulator.getVelocity().in(MetersPerSecond));
        }
      default:
        throw new IllegalArgumentException("FeedForward controller not set");
    }
  }

  public void setTargetPosition(Distance position) {
    this.position = position;
  }

  /** Final position */
  public Distance getTargetPosition() {
    return position;
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
    return Math.abs(manipulator.getPosition().minus(position).in(Meters)) < pidConfig.linearTolerance.in(Meters);
  }

  public void setEndOnTarget(boolean endOnTarget) {
    this.endOnTarget = endOnTarget;
  }

  protected void synchronizePIDSettings() {
    pidConfig.applyTo(pid, MetersPerSecond);
  }

  /** In terms of Meters and Meters/sec */
  public TrapezoidProfile.State getSetpoint() {
    return pid.getSetpoint();
  }
}
