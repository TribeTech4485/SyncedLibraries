package frc.robot.SyncedLibraries.SystemBases;

import static edu.wpi.first.units.Units.Radians;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorAngleCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorFFAngleCommand;
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
 * 4.) Run the {@link #setPositionPID(ManipulatorAngleCommand)} methods
 * in the constructor to set the PID values.
 * <p>
 * 5.) Adjust position multipliers to be in radians
 * <p>
 * You also can use standard {@link SubsystemBase} and {@link ManipulatorBase}
 * methods
 * <p>
 * Add a loop to run all tests to Robot.testInit()
 */
public abstract class AngleManipulatorBase extends ManipulatorBase {
  protected Angle maxPosition;
  protected Angle minPosition;
  protected final PIDConfig pidConfig;

  /** If null, run the {@link #setPositionPID(double, double, double, double)} */
  protected final ManipulatorAngleCommand moveCommand;

  public AngleManipulatorBase(PIDConfig pidConfig, ManipulatorFFAngleCommand.FeedForwardType feedForwardType) {
    this.pidConfig = pidConfig;
    moveCommand = new ManipulatorFFAngleCommand(this, getAngle(), pidConfig, feedForwardType);
  }

  public void _setAngle(Angle angle) {
    for (RelativeEncoder encoder : encoders) {
      encoder.setPosition(angle.in(Radians));
    }
  }

  public Angle getAngle() {
    double average = 0;
    for (RelativeEncoder encoder : encoders) {
      average += encoder.getPosition();
    }
    return Radians.of(average / encoders.size());
  }

  /**
   * Set the max and min position of the manipulator
   * <p>
   * If set out of bounds then the manipulator will move to limit when called
   */
  public void setAngleBounds(Angle min, Angle max) {
    minPosition = min;
    maxPosition = max;
  }

  public ManipulatorAngleCommand getMoveCommand() {
    return moveCommand;
  }

  /** Get the target position of the manipulator */
  public Angle getTargetPosition() {
    return moveCommand.getTargetPosition();
  }

  public PIDConfig getPIDConfig() {
    return pidConfig;
  }

  /**
   * Move the manipulator to a position in radians or meters
   * <p>
   * <b>STOPS COMMANDS</b>
   */
  public void moveToPosition(Angle angle) {
    moveToPosition(angle, true);
  }

  /**
   * Move the manipulator to a position in radians or meters
   * <p>
   * <b>Only run with stopCommands false if the position command is known to be
   * running</b>
   */
  public void moveToPosition(Angle angle, boolean stopCommands) {
    if (stopCommands) {
      stopCommand();
    }
    if (angle.compareTo(maxPosition) > 0) {
      angle = maxPosition;
      System.out.println("ManipulatorBase: Position out of bounds" + angle + " > " + maxPosition);
    } else if (angle.compareTo(minPosition) < 0) {
      angle = minPosition;
      System.out.println("ManipulatorBase: Position out of bounds" + angle + " < " + minPosition);
    }
    moveCommand.setTargetPosition(angle);
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
  }

  @Override
  public void stopCommand() {
    cancelMoveToPosition();
  }

  /** Set multiplier to convert from encoder values to radians on manipulator */
  public void setPositionMultiplier(double multiplier) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(multiplier)),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }
}