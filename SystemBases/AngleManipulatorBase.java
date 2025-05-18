package frc.robot.SyncedLibraries.SystemBases;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorAngleCommand;
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
 * 4.) Adjust position multipliers using
 * {@link ManipulatorBase#setEncoderMultiplier(double)} to be in
 * radians
 * <p>
 * 5.) Set {@link #setBreakerMaxAmps(int)} and {@link #setCurrentLimit(int)}
 * current limits
 * <p>
 * 6.) Set the angle limits using {@link #setAngleBounds(Angle, Angle)}
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

  public AngleManipulatorBase(PIDConfig pidConfig) {
    this.pidConfig = pidConfig;
    moveCommand = new ManipulatorAngleCommand(this, getAngle(), pidConfig);
  }

  /**
   * Sets the current angle that the manipulator is at. USE CAUTIOUSLY
   */
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
    return Radians.of((average / encoders.size()) % (2 * Math.PI));
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
    // if (moveCommand != null) {
    // if (moveCommand.isScheduled()) {
    moveCommand.cancel();
    System.out.println("ManipulatorBase: Cancelling move command");
    // }
    // }
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putBoolean(getName() + " At Position", isAtPosition());
    SmartDashboard.putNumber(getName() + " Current Angle (deg)", getAngle().in(Degrees));
    SmartDashboard.putNumber(getName() + " Target Angle (deg)", getTargetPosition().in(Degrees));
    SmartDashboard.putNumber(getName() + " Setpoint Angle (deg)",
        Units.radiansToDegrees(getMoveCommand().getSetpoint().position));
    SmartDashboard.putNumber(getName() + " Current Encoder value", getEncoder(0).getPosition());
  }

  @Override
  public void stopCommand() {
    cancelMoveToPosition();
  }
}