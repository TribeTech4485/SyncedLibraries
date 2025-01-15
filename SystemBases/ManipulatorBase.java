package frc.robot.SyncedLibraries.SystemBases;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.LinkedList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/**
 * <strong> To impliment: </strong>
 * <p>
 * 1.) Create class that extends this class
 * <p>
 * 2.) Override the homing method and any other methods you need
 * <p>
 * 3.) Add the motors
 * <p>
 * 4.) Run the set___PID methods
 * in the constructor to set the PID values.
 * <p>
 * 5.) Adjust speed and position multipliers if needed
 * <p>
 * You also can use standard subsystem methods
 */
public abstract class ManipulatorBase extends Estopable {
  /**
   * @deprecated Manually adding is unneccesary, now handled by the
   *             ESTOPPABLE parent class
   */
  @Deprecated
  public static LinkedList<Estopable> allManipulators = new LinkedList<Estopable>();
  /** If null, run the {@link #setPositionPID(double, double, double, double)} */
  protected ManipulatorMoveCommand moveCommand;
  /** If null, run the {@link #setSpeedPID(double, double, double, double)} */
  protected ManipulatorSpeedCommand speedCommand;
  protected double positionMultiplier = 1;
  protected double speedMultiplier = 1;
  protected double maxPower = 1;
  protected double maxPosition = Double.MAX_VALUE;
  protected double minPosition = -Double.MAX_VALUE;
  protected LinkedList<CANSparkMax> motors = new LinkedList<CANSparkMax>();
  protected LinkedList<RelativeEncoder> encoders = new LinkedList<RelativeEncoder>();
  protected BooleanSupplier customSensor = () -> false;

  // ===================== Positional Methods ===================== //
  /** Get the known position of the manipulator in degrees */
  public double getPosition() {
    double average = 0;
    for (RelativeEncoder encoder : encoders) {
      average += encoder.getPosition();
    }
    return (average / encoders.size()) * positionMultiplier;
  }

  /**
   * Set the known position of the manipulator in degrees.
   * <p>
   * USE SPARINGLY
   */
  public void _setPosition(double position) {
    for (RelativeEncoder encoder : encoders) {
      encoder.setPosition(position / positionMultiplier);
    }
  }

  /** Set multiplier to convert from encoder values to degrees on manipulator */
  public void setPositionMultiplier(double multiplier) {
    positionMultiplier = multiplier;
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
   * Move the manipulator to a position in degrees or meters
   * <p>
   * <b>STOPS COMMANDS</b>
   */
  public void moveToPosition(double position) {
    moveToPosition(position, true);
  }

  /**
   * Move the manipulator to a position in degrees or meters
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
    return moveCommand == null ? moveCommand.atPosition : false;
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
    for (CANSparkMax motor : motors) {
      average += motor.get();
    }
    return average / motors.size();
  }

  /** Same as {@link #getAvePower()} but ABSed before averaging */
  public double getAbsPower() {
    double average = 0;
    for (CANSparkMax motor : motors) {
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
    percent = Math.min(maxPower, Math.max(-maxPower, percent));
    for (CANSparkMax motor : motors) {
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

  public void setMaxPower(double maxPower) {
    this.maxPower = maxPower;
  }

  /** Get the average voltage of the motors */
  public double getAveVoltage() {
    double average = 0;
    for (CANSparkMax motor : motors) {
      average += motor.getBusVoltage();
    }
    return average / motors.size();
  }

  /** Same as {@link #getAveVoltage()} but ABSed before averaging */
  public double getAbsVoltage() {
    double average = 0;
    for (CANSparkMax motor : motors) {
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
    for (CANSparkMax motor : motors) {
      motor.setVoltage(voltage);
    }
  }

  // ===================== Speed Methods ===================== //

  /** Set multiplier to convert from encoder values to rpm on manipulator */
  public void setSpeedMultiplier(double multiplier) {
    speedMultiplier = multiplier;
  }

  /** Get the live speed of the motor in rpm */
  public double getCurrentSpeed() {
    double average = 0;
    for (RelativeEncoder encoder : encoders) {
      average += encoder.getVelocity();
    }
    return (average / encoders.size()) * speedMultiplier;
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
    return speedCommand == null ? speedCommand.atSpeed : false;
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
    for (CANSparkMax motor : motors) {
      encoders.add(motor.getEncoder());
    }
  }

  /** Do I really need to explain this? */
  public void addMotors(CANSparkMax... motors) {
    for (CANSparkMax motor : motors) {
      this.motors.add(motor);
    }
    updateEncoders();
  }

  public CANSparkMax getMotor(int index) {
    return motors.get(index);
  }

  public RelativeEncoder getEncoder(int index) {
    return encoders.get(index);
  }

  public CANSparkMax[] getMotors() {
    return motors.toArray(new CANSparkMax[0]);
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
    for (CANSparkMax motor : motors) {
      motor.stopMotor();
    }
  }

  /** Not available on all setups */
  public void setBrakeMode(boolean brakeOnStop) {
    for (CANSparkMax motor : motors) {
      motor.setIdleMode(brakeOnStop ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
  }

  /** Use to invert all motor functions, including encoders */
  public void setInverted(boolean inverted) {
    for (CANSparkMax motor : motors) {
      motor.setInverted(inverted);
      motor.getEncoder().setInverted(inverted);
    }
  }

  public void setCurrentLimit(int limit) {
    for (CANSparkMax motor : motors) {
      motor.setSmartCurrentLimit(limit);
    }
  }

  public void setRampRate(double rate) {
    for (CANSparkMax motor : motors) {
      motor.setOpenLoopRampRate(rate);
      motor.setClosedLoopRampRate(rate);
    }
  }

  /** Use to invert specific motors {@link #setInverted(boolean)} */
  public void invertSpecificMotors(boolean inverted, int... motorIndexes) {
    for (int index : motorIndexes) {
      motors.get(index).setInverted(inverted);
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
    // new Throwable("Stopping commands").printStackTrace();
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

  /**
   * This will always be silently called upon initialization of any manipulator
   * subclass, adds itself to the list of all manipulators for emergency stop
   */
  public ManipulatorBase() {
    allManipulators.add(this);
  }
}
