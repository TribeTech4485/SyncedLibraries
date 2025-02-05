package frc.robot.SyncedLibraries.SystemBases;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import java.util.LinkedList;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

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
 * 4.) Run the set___PID methods
 * in the constructor to set the PID values.
 * <p>
 * 5.) Adjust speed and position multipliers if needed
 * <p>
 * You also can use standard {@link SubsystemBase} methods
 * <p>
 * Add a loop to run all tests to Robot.testInit()
 */
public abstract class ManipulatorBase extends Estopable {
  /** If null, run the {@link #setPositionPID(double, double, double, double)} */
  protected ManipulatorMoveCommand moveCommand;
  /** If null, run the {@link #setSpeedPID(double, double, double, double)} */
  protected ManipulatorSpeedCommand speedCommand;
  protected int breakerMaxAmps = 20;
  protected double maxPosition = Double.MAX_VALUE;
  protected double minPosition = -Double.MAX_VALUE;
  protected LinkedList<SparkMax> motors = new LinkedList<SparkMax>();
  protected LinkedList<RelativeEncoder> encoders = new LinkedList<RelativeEncoder>();
  protected BooleanSupplier customSensor = () -> false;

  // ===================== Positional Methods ===================== //
  /** Get the known position of the manipulator in radians/meters */
  public double getPosition() {
    double average = 0;
    for (RelativeEncoder encoder : encoders) {
      average += encoder.getPosition();
    }
    return (average / encoders.size());
  }

  /**
   * Set the known position of the manipulator in degrees.
   * <p>
   * USE SPARINGLY
   */
  public void _setPosition(double position) {
    for (RelativeEncoder encoder : encoders) {
      encoder.setPosition(position);
    }
  }

  /** Set multiplier to convert from encoder values to degrees on manipulator */
  public void setPositionMultiplier(double multiplier) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(multiplier)),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  /**
   * Set the max and min position of the manipulator
   * <p>
   * If set out of bounds then the manipulator will move to limit when called
   */
  public void setPositionBounds(double min, double max) {
    minPosition = min;
    maxPosition = max;
  }

  public void setPositionPID(double kP, double kI, double kD, double tolerance) {
    cancelMoveToPosition();
    this.moveCommand = new ManipulatorMoveCommand(this, Integer.MAX_VALUE, tolerance, kP, kI, kD);
  }

  public void setPositionPID(ManipulatorMoveCommand command) {
    cancelMoveToPosition();
    this.moveCommand = command;
  }

  public ManipulatorMoveCommand getMoveCommand() {
    return moveCommand;
  }

  /** Get the target position of the manipulator in degrees */
  public double getTargetPosition() {
    if (moveCommand == null) {
      return Integer.MAX_VALUE;
    }
    return moveCommand.getTargetPosition();
  }

  /**
   * Move the manipulator to a position in radians or meters
   * <p>
   * <b>STOPS COMMANDS</b>
   */
  public void moveToPosition(double position) {
    moveToPosition(position, true);
  }

  /**
   * Move the manipulator to a position in radians or meters
   * <p>
   * <b>Only run with stopCommands false if the position command is known to be
   * running</b>
   */
  public void moveToPosition(double position, boolean stopCommands) {
    if (stopCommands) {
      stopCommands();
    }
    if (position > maxPosition) {
      position = maxPosition;
      System.out.println("ManipulatorBase: Position out of bounds" + position + " > " + maxPosition);
    } else if (position < minPosition) {
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
    return moveCommand == null ? false : moveCommand.atPosition;
  }

  public final void cancelMoveToPosition() {
    if (moveCommand != null && moveCommand.isScheduled()) {
      CommandScheduler.getInstance().cancel(moveCommand);
      System.out.println("ManipulatorBase: Cancelling move command");
    }
  }

  // ===================== Raw Speed Methods ===================== //
  /** Get the raw speed of the motor in the range -1 to 1 */
  public double getAvePower() {
    double average = 0;
    for (SparkMax motor : motors) {
      average += motor.get();
    }
    return average / motors.size();
  }

  /** Same as {@link #getAvePower()} but ABSed before averaging */
  public double getAbsPower() {
    double average = 0;
    for (SparkMax motor : motors) {
      average += Math.abs(motor.get());
    }
    return average / motors.size();
  }

  /**
   * Set the raw speed of the motor in the range -1 to 1
   * <p>
   * Used for manual control
   */
  public void setPower(double percent, boolean stopCommands) {
    if (stopCommands) {
      stopCommands();
    }
    for (SparkMax motor : motors) {
      motor.set(percent);
    }
  }

  /**
   * Set the raw power of the motor in the range -1 to 1
   * <p>
   * Used for manual control
   * <p>
   * <b>STOPS COMMANDS</b>
   */
  public void setPower(double percent) {
    setPower(percent, true);
  }

  /** Get the average voltage of the motors */
  public double getAveVoltage() {
    double average = 0;
    for (SparkMax motor : motors) {
      average += motor.getBusVoltage();
    }
    return average / motors.size();
  }

  /** Same as {@link #getAveVoltage()} but ABSed before averaging */
  public double getAbsVoltage() {
    double average = 0;
    for (SparkMax motor : motors) {
      average += Math.abs(motor.getBusVoltage());
    }
    return average / motors.size();
  }

  /**
   * Set the raw speed of the motor in the range -1 to 1
   * <p>
   * Used for manual control
   */
  public void setVoltage(double voltage, boolean stopCommands) {
    if (stopCommands) {
      stopCommands();
    }

    // double line = motors.get(0).getBusVoltage();
    // voltage = Math.min(line, Math.max(-line, voltage));
    for (SparkMax motor : motors) {
      motor.setVoltage(voltage);
    }
  }

  // ===================== Speed Methods ===================== //

  /** Set multiplier to convert from encoder values to rpm on manipulator */
  public void setSpeedMultiplier(double multiplier) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().apply(new EncoderConfig().velocityConversionFactor(multiplier)),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  /** Get the live speed of the motor in rpm */
  public double getCurrentSpeed() {
    double average = 0;
    for (RelativeEncoder encoder : encoders) {
      average += encoder.getVelocity();
    }
    return (average / encoders.size());
  }

  /** Get the target speed of the motor in rpm */
  public double getTargetSpeed() {
    if (speedCommand == null) {
      return Integer.MAX_VALUE;
    }
    return speedCommand.getTargetSpeed();
  }

  /** Set the speed of the motor in rpm */
  public void setTargetSpeed(double rpm) {
    stopCommands();
    speedCommand.setTargetSpeed(rpm);
    speedCommand.schedule();
  }

  public ManipulatorSpeedCommand getSpeedCommand() {
    return speedCommand;
  }

  public boolean isAtSpeed() {
    return speedCommand == null ? false : speedCommand.atSpeed;
  }

  public void setSpeedPID(double kP, double kI, double kD, double tolerance) {
    cancelSpeedCommand();
    this.speedCommand = new ManipulatorSpeedCommand(this, Integer.MAX_VALUE, tolerance, kP, kI, kD);
  }

  public final void cancelSpeedCommand() {
    if (speedCommand != null && speedCommand.isScheduled()) {
      CommandScheduler.getInstance().cancel(speedCommand);
      System.out.println("ManipulatorBase: Cancelling speed command");
    }
  }

  // ===================== Utility Methods ===================== //

  /** Adds all the encoders from the motors into the encoders list */
  protected void updateEncoders() {
    encoders.clear();
    for (SparkMax motor : motors) {
      encoders.add(motor.getEncoder());
    }
  }

  /** Do I really need to explain this? */
  public void addMotors(SparkMax... motors) {
    for (SparkMax motor : motors) {
      this.motors.add(motor);
    }
    updateEncoders();
  }

  public SparkMax getMotor(int index) {
    return motors.get(index);
  }

  public RelativeEncoder getEncoder(int index) {
    return encoders.get(index);
  }

  public SparkMax[] getMotors() {
    return motors.toArray(new SparkMax[0]);
  }

  public RelativeEncoder[] getEncoders() {
    return encoders.toArray(new RelativeEncoder[0]);
  }

  /**
   * Stop all motors
   * <p>
   * DOES NOT STOP COMMANDS
   */
  public void stop() {
    for (SparkMax motor : motors) {
      motor.stopMotor();
    }
  }

  /** Not available on all setups */
  public void setBrakeMode(boolean brakeOnStop) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().idleMode(brakeOnStop ? IdleMode.kBrake : IdleMode.kCoast),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  /** Use to invert all motor functions, including encoders */
  public void setInverted(boolean inverted) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().inverted(inverted)
          .apply(new EncoderConfig().inverted(inverted)), ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
  }

  public void setCurrentLimit(int limit) {
    if (limit > breakerMaxAmps) {
      DriverStation.reportWarning(
          "ManipulatorBase: Current limit out of bounds" + limit + " > " + breakerMaxAmps, true);
      if (limit > 1.25 * breakerMaxAmps) {
        limit = (int) (1.25 * breakerMaxAmps);
      }
    }
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().smartCurrentLimit(limit), ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
  }

  /** To prevent setting to draw too many amps */
  public void setBreakerMaxAmps(int amps) {
    breakerMaxAmps = amps;
  }

  public void setRampRate(double rate) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig()
          .closedLoopRampRate(rate)
          .openLoopRampRate(rate),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  /** Use to invert specific motors {@link #setInverted(boolean)} */
  public void invertSpecificMotors(boolean inverted, int... motorIndexes) {
    for (int index : motorIndexes) {
      motors.get(index).configure(new SparkMaxConfig().inverted(inverted),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      // motors.get(index).getEncoder().setInverted(inverted);
    }
  }

  /** Is power going to any motors */
  public boolean isRunning() {
    return getAbsPower() != 0;
  }

  public boolean isSpinning() {
    return getCurrentSpeed() != 0;
  }

  public void stopCommands() {
    cancelMoveToPosition();
    cancelSpeedCommand();
  }

  public void fullStop() {
    stopCommands();
    stop();
  }

  /**
   * Move all motors to known position and then reset encoders
   * <p>
   * Do it yourself
   */
  public Command home() {
    return new PrintCommand("Homing not implemented on subsystem " + getName() + "... Continuing");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + " Position", getPosition());
    SmartDashboard.putNumber(getName() + " Speed", getCurrentSpeed());
    SmartDashboard.putNumber(getName() + " Power", getAvePower());
    SmartDashboard.putBoolean(getName() + " At Speed/Power", isAtSpeed() || isAtPosition());
  }

  /**
   * When put into test mode, this command will run
   * <p>
   * Basically run all systems to show they work
   */
  public abstract Command test();

  public static ManipulatorBase[] getAllManipulators() {
    Estopable[] estopables = Estopable.getAllEstopables();
    LinkedList<ManipulatorBase> manipulators = new LinkedList<ManipulatorBase>();
    for (Estopable estopable : estopables) {
      if (estopable instanceof ManipulatorBase) {
        manipulators.add((ManipulatorBase) estopable);
      }
    }
    return manipulators.toArray(new ManipulatorBase[0]);
  }

  @Override
  public void onDisable() {
    // Save motor config
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig(), ResetMode.kNoResetSafeParameters,
          PersistMode.kPersistParameters);
    }
  }

  public ManipulatorBase() {
    // Make sure the amp limit is reasonable
    for (SparkMax motor : motors) {
      if (motor.configAccessor.getSmartCurrentLimit() > breakerMaxAmps) {
        motor.configure(new SparkMaxConfig().smartCurrentLimit(breakerMaxAmps), ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
      }
    }
  }
}
