package frc.robot.SyncedLibraries.SystemBases.Utils;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.SystemBases.AngleManipulatorBase;

/**
 * Nearly identical to the ManipulatorMoveCommand, but with FeedForward
 * controllers
 */
public class ManipulatorFFAngleCommand extends ManipulatorAngleCommand {
  protected FeedForwardType feedForwardType;

  private ProfiledPIDController pidProfiled;

  /**
   * Just like {@link ManipulatorDistanceCommand}, but with a selectable
   * FeedForward
   * 
   * @param manipulator     The manipulator to control
   * @param position        The position to move to
   * @param tolerance       The tolerance for the position
   * @param kP              The proportional gain
   * @param kI              The integral gain
   * @param kD              The derivative gain
   * @param feedForwardType The type of feedforward controller to use
   *                        (Simple, Elevator, Arm)
   * @param kS              The static gain
   * @param kV              The velocity gain
   * @param kG              The gravity gain (leave at 0 if not using Arm or
   *                        Elevator)
   * @param kA              The acceleration gain (generally assumed to be 0)
   * @param maxVelocity     The maximum velocity of the manipulator
   * @param maxAcceleration The maximum acceleration of the manipulator
   */
  public ManipulatorFFAngleCommand(AngleManipulatorBase manipulator, Angle position,
      PIDConfig pidConfig, FeedForwardType feedForwardType) {
    super(manipulator, position, pidConfig);

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        pidConfig.maxAngularVelocity.in(RadiansPerSecond),
        pidConfig.maxAngularAcceleration.in(RadiansPerSecondPerSecond));
    pidProfiled = new ProfiledPIDController(pidConfig.P, pidConfig.I, pidConfig.D, constraints);
    pidProfiled.enableContinuousInput(-0 * Math.PI, 2 * Math.PI);

    if (feedForwardType == null) {
      feedForwardType = FeedForwardType.SimpleMotor;
      DriverStation.reportWarning("FeedForwardType not set, defaulting to SimpleMotor", true);
    }
    this.feedForwardType = feedForwardType;
  }

  private double getFF() {
    switch (feedForwardType) {
      case SimpleMotor:
        if (pidConfig.A == 0.0) {
          return pidConfig.S * Math.signum(pidProfiled.getSetpoint().velocity) + pidConfig.V * pidProfiled
              .getSetpoint().velocity;
        } else {
          double A = -pidConfig.V / pidConfig.A;
          double B = 1.0 / pidConfig.A;
          double A_d = Math.exp(A * 0.02);
          double B_d = 1.0 / A * (A_d - 1.0) * B;
          return pidConfig.S * Math.signum(manipulator.getCurrentSpeed().in(RadiansPerSecond))
              + 1.0 / B_d * (pidProfiled.getSetpoint().velocity - A_d
                  * manipulator.getCurrentSpeed().in(RadiansPerSecond));
        }

      case Elevator:
        if (pidConfig.A == 0.0) {
          return pidConfig.S * Math.signum(pidProfiled.getSetpoint().velocity) + pidConfig.G
              + pidConfig.V * pidProfiled.getSetpoint().velocity;
        } else {
          double A = -pidConfig.V / pidConfig.A;
          double B = 1.0 / pidConfig.A;
          double A_d = Math.exp(A * 0.02);
          double B_d = 1.0 / A * (A_d - 1.0) * B;
          return pidConfig.G
              + pidConfig.S * Math.signum(manipulator.getCurrentSpeed().in(RadiansPerSecond))
              + 1.0 / B_d
                  * (pidProfiled.getSetpoint().velocity - A_d * manipulator.getCurrentSpeed().in(RadiansPerSecond));
        }

      case Arm:
        if (pidConfig.A == 0.0) {
          return pidConfig.S * Math.signum(pidProfiled.getSetpoint().velocity) +
              pidConfig.G * Math.cos(manipulator.getAngle().in(Radians)) +
              pidConfig.V * pidProfiled.getSetpoint().velocity;
        } else {
          double A = -pidConfig.V / pidConfig.A;
          double B = 1.0 / pidConfig.A;
          double A_d = Math.exp(A * 0.02);
          double B_d = 1.0 / A * (A_d - 1.0) * B;
          return pidConfig.G * Math.cos(manipulator.getAngle().in(Radians))
              + pidConfig.S * Math.signum(manipulator.getCurrentSpeed().in(RadiansPerSecond))
              + 1.0 / B_d
                  * (pidProfiled.getSetpoint().velocity - A_d * manipulator.getCurrentSpeed().in(RadiansPerSecond));
        }
      default:
        throw new IllegalArgumentException("FeedForward controller not set");
    }
  }

  @Override
  public void initialize() {
    manipulator.stop();
    pidProfiled.reset(new TrapezoidProfile.State(manipulator.getAngle().in(Radians),
        manipulator.getCurrentSpeed().in(RadiansPerSecond)));
    pidProfiled.setGoal(targetPosition.in(Radians));
  }

  @Override
  public void execute() {
    pidProfiled.setGoal(targetPosition.in(Radians) % (2 * Math.PI));
    manipulator.setVoltage(
        Volts.of(pidProfiled.calculate(manipulator.getAngle().in(Radians) % (2 * Math.PI)) + getFF()),
        false);
    atPosition = isAtPosition();
  }

  public Angle getSetpoint() {
    return Radians.of(pidProfiled.getSetpoint().position);
  }

  public enum FeedForwardType {
    SimpleMotor, Elevator, Arm
  }
}
