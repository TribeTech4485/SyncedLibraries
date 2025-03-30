package frc.robot.SyncedLibraries.SystemBases.Utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.SyncedLibraries.SystemBases.PositionManipulatorBase;

// TODO: PURGE THIS FILE, move to ManipulatorDistanceCommand

/**
 * Nearly identical to the ManipulatorMoveCommand, but with FeedForward
 * controllers
 */
public class ManipulatorFFDistanceCommand extends ManipulatorDistanceCommand {
  protected SimpleMotorFeedforward simpleController;
  protected ElevatorFeedforward elevatorController;
  protected ArmFeedforward armController;

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
  public ManipulatorFFDistanceCommand(PositionManipulatorBase manipulator, Distance position, PIDConfig pidConfig,
      FeedForwardType feedForwardType) {
    super(manipulator, position, pidConfig);

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        pidConfig.maxLinearVelocity.in(MetersPerSecond),
        pidConfig.maxLinearAcceleration.in(MetersPerSecondPerSecond));
    pidProfiled = new ProfiledPIDController(pidConfig.P, pidConfig.I, pidConfig.D, constraints);
    pidProfiled.setTolerance(tolerance.in(Meters));

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
          return pidConfig.S * Math.signum(manipulator.getVelocity().in(MetersPerSecond))
              + 1.0 / B_d * (pidProfiled.getSetpoint().velocity - A_d
                  * manipulator.getVelocity().in(MetersPerSecond));
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
              + pidConfig.S * Math.signum(manipulator.getVelocity().in(MetersPerSecond))
              + 1.0 / B_d
                  * (pidProfiled.getSetpoint().velocity - A_d * manipulator.getVelocity().in(MetersPerSecond));
        }

      case Arm:
        if (pidConfig.A == 0.0) {
          return pidConfig.S * Math.signum(pidProfiled.getSetpoint().velocity) +
              pidConfig.G * Math.cos(manipulator.getPosition().in(Meters)) +
              pidConfig.V * pidProfiled.getSetpoint().velocity;
        } else {
          double A = -pidConfig.V / pidConfig.A;
          double B = 1.0 / pidConfig.A;
          double A_d = Math.exp(A * 0.02);
          double B_d = 1.0 / A * (A_d - 1.0) * B;
          return pidConfig.G * Math.cos(manipulator.getPosition().in(Meters))
              + pidConfig.S * Math.signum(manipulator.getVelocity().in(MetersPerSecond))
              + 1.0 / B_d
                  * (pidProfiled.getSetpoint().velocity - A_d * manipulator.getVelocity().in(MetersPerSecond));
        }
      default:
        throw new IllegalArgumentException("FeedForward controller not set");
    }
  }

  @Override
  public void initialize() {
    manipulator.stop();
    pidProfiled.reset(new TrapezoidProfile.State(manipulator.getPosition().in(Meters),
        manipulator.getVelocity().in(MetersPerSecond)));
    pidProfiled.setGoal(position.in(Meters));
  }

  @Override
  public void execute() {
    pidProfiled.setGoal(position.in(Meters));
    manipulator.setVoltage(Volts.of(pidProfiled.calculate(manipulator.getPosition().in(Meters)) + getFF()),
        false);
    atPosition = isAtPosition();
  }

  public enum FeedForwardType {
    SimpleMotor, Elevator, Arm
  }

  public ProfiledPIDController getController() {
    return pidProfiled;
  }
}
