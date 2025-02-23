package frc.robot.SyncedLibraries.SystemBases;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorDistanceCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorFFDistanceCommand;
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
 * 4.) Run the {@link #setPositionPID(ManipulatorDistanceCommand)} methods
 * in the constructor to set the PID values.
 * <p>
 * 5.) Adjust position multipliers to be in meters
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

  public PositionManipulatorBase(PIDConfig pidConfig, ManipulatorFFDistanceCommand.FeedForwardType feedForwardType) {
    this.pidConfig = pidConfig;
    moveCommand = new ManipulatorFFDistanceCommand(this, getPosition(), pidConfig, feedForwardType);
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
    SmartDashboard.putNumber(getName() + "position (m)", getPosition().in(Meters));
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

  /** Set multiplier to convert from encoder values to meters on manipulator */
  public void setPositionMultiplier(double multiplier) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(multiplier)),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }
}