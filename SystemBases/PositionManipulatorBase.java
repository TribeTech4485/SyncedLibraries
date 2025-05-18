package frc.robot.SyncedLibraries.SystemBases;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorDistanceCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

/**
 * <strong> To impliment: </strong>
 * <p>
 * 1.) Create class that extends this class
 * <p>
 * 2.) Override the {@link #ESTOP()} and {@link #test()} method and any other
 * methods you need
 * <p>
 * 3.) Add the motors
 * <p>
 * 4.) Adjust position multipliers to be in meters using
 * {@link ManipulatorBase#setEncoderMultiplier(double)}
 * <p>
 * 5.) Set {@link #setBreakerMaxAmps(int)} and {@link #setCurrentLimit(int)}
 * current limits
 * <p>
 * 6.) Set the position limits using
 * {@link #setPositionBounds(Distance, Distance)}
 * <p>
 * 7.) Run {@link #persistMotorConfig()} to save the motor config
 * <p>
 * You also can use standard {@link SubsystemBase} and {@link ManipulatorBase}
 * methods
 * <p>
 * Add a loop to run all tests to Robot.testInit()
 */
public abstract class PositionManipulatorBase extends ManipulatorBase {
  protected Distance maxPosition;
  protected Distance minPosition;
  protected final PIDConfig pidConfig;

  /** If null, run the {@link #setPositionPID(double, double, double, double)} */
  protected final ManipulatorDistanceCommand moveCommand;

  /**
   * Create a manipulator with a PID controller
   * 
   * @param pidConfig       The PID values
   * @param feedForwardType The type of feedforward controller to use, if unknown
   *                        use SimpleMotor
   */
  public PositionManipulatorBase(PIDConfig pidConfig) {
    this.pidConfig = pidConfig;
    moveCommand = new ManipulatorDistanceCommand(this, getPosition(), pidConfig);
  }

  public Distance getPosition() {
    double average = 0;
    for (RelativeEncoder encoder : encoders) {
      average += encoder.getPosition();
    }
    return Meters.of(average / encoders.size());
  }

  public LinearVelocity getVelocity() {
    double average = 0;
    for (RelativeEncoder encoder : encoders) {
      average += encoder.getVelocity();
    }
    return MetersPerSecond.of(average / encoders.size());
  }

  /**
   * Set the max and min position of the manipulator
   * <p>
   * If set out of bounds then the manipulator will move to limit when called
   */
  public void setPositionBounds(Distance min, Distance max) {
    minPosition = min;
    maxPosition = max;
  }

  public PIDConfig getPIDConfig() {
    return pidConfig;
  }

  public ManipulatorDistanceCommand getMoveCommand() {
    return moveCommand;
  }

  /** Get the target position of the manipulator */
  public Distance getTargetPosition() {
    return moveCommand.getTargetPosition();
  }

  /**
   * Move the manipulator to a position
   * <p>
   * <b>STOPS COMMANDS</b>
   */
  public void moveToPosition(Distance position) {
    moveToPosition(position, true);
  }

  /**
   * Move the manipulator to a position
   * <p>
   * <b>Only run with stopCommands false if the position command is known to be
   * running</b>
   */
  public void moveToPosition(Distance position, boolean stopCommands) {
    if (stopCommands) {
      stopCommand();
    }
    if (position.compareTo(maxPosition) > 0) {
      position = maxPosition;
      System.out.println("ManipulatorBase: Position out of bounds" + position + " > " + maxPosition);
    } else if (position.compareTo(minPosition) < 0) {
      position = minPosition;
      System.out.println("ManipulatorBase: Position out of bounds" + position + " < " + minPosition);
    }
    moveCommand.setTargetPosition(position);
    if (stopCommands || !moveCommand.isScheduled()) {
      moveCommand.schedule();
    }
  }

  /** If moveCommand not initialized returns false */
  public boolean isAtPosition() {
    return moveCommand.atPosition;
  }

  public final void cancelMoveToPosition() {
    if (moveCommand != null) {
      if (moveCommand.isScheduled()) {
        CommandScheduler.getInstance().cancel(moveCommand);
        System.out.println("ManipulatorBase: Cancelling move command");
      }
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putBoolean(getName() + " At Position", isAtPosition());
    SmartDashboard.putNumber(getName() + " current position (m)", getPosition().in(Meters));
    SmartDashboard.putNumber(getName() + " target position (m)", getTargetPosition().in(Meters));
    SmartDashboard.putNumber(getName() + " current velocity (m/s)", getVelocity().in(MetersPerSecond));
    SmartDashboard.putNumber(getName() + " setpoint velocity (m/s)", moveCommand.getSetpoint().velocity);
    SmartDashboard.putNumber(getName() + " setpoint position (m)", moveCommand.getSetpoint().position);
  }

  @Override
  public void stopCommand() {
    cancelMoveToPosition();
  }

  /**
   * Set the known position of the manipulator.
   * <p>
   * USE SPARINGLY
   */
  public void _setPosition(Distance position) {
    for (RelativeEncoder encoder : encoders) {
      encoder.setPosition(position.in(Meters));
    }
  }
}