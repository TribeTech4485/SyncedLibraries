package frc.robot.SyncedLibraries.SystemBases.Utils;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.SyncedLibraries.SystemBases.AngleManipulatorBase;

/**
 * Nearly identical to the ManipulatorMoveCommand, but with FeedForward
 * controllers
 */
public class ManipulatorFFAngleCommand extends ManipulatorAngleCommand {
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
  public ManipulatorFFAngleCommand(AngleManipulatorBase manipulator, Angle position,
      Angle tolerance, double kP, double kI, double kD, FeedForwardType feedForwardType,
      double kS, double kV, double kG, double kA, AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    super(manipulator, position, tolerance, kP, kI, kD);

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        maxVelocity.in(RadiansPerSecond), maxAcceleration.in(RadiansPerSecondPerSecond));
    pidProfiled = new ProfiledPIDController(kP, kI, kD, constraints);
    pidProfiled.setTolerance(tolerance.in(Radians));

    switch (feedForwardType) {
      case SimpleMotor:
        simpleController = new SimpleMotorFeedforward(kS, kV, kA);
        break;
      case Elevator:
        elevatorController = new ElevatorFeedforward(kS, kG, kV, kA);
        break;
      case Arm:
        armController = new ArmFeedforward(kS, kG, kV, kA);
        break;
      default:
        System.out.println("Invalid FeedForward type for manipulator \"" + manipulator.getName()
            + "\", defaulting to Simple. The options are \"Simple\", \"Elevator\", and \"Arm\"");
        simpleController = new SimpleMotorFeedforward(kS, kV);
        break;
    }
    this.feedForwardType = feedForwardType;
  }

  private double getFF() {
    switch (feedForwardType) {
      case SimpleMotor:
        return simpleController.calculate(pidProfiled.getSetpoint().velocity);
      case Elevator:
        return elevatorController.calculate(pidProfiled.getSetpoint().velocity);
      case Arm:
        return armController.calculate(pidProfiled.getSetpoint().position, pidProfiled.getSetpoint().velocity);
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
    manipulator.setVoltage(Volts.of(pidProfiled.calculate(manipulator.getAngle().in(Radians)) + getFF()),
        false);
    atPosition = isAtPosition();
  }

  public enum FeedForwardType {
    SimpleMotor, Elevator, Arm
  }
}
