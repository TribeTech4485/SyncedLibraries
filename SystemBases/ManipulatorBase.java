package frc.robot.SyncedLibraries.SystemBases;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
public class ManipulatorBase extends SubsystemBase {
  ManipulatorMoveCommand moveCommand;
  ManipulatorSpeedCommand speedCommand;
  double positionMultiplier = 1;
  double speedMultiplier = 1;
  ArrayList<CANSparkMax> motors;
  ArrayList<RelativeEncoder> encoders;

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

  public void setPositionPID(double kP, double kI, double kD, double tolerance) {
    cancelMoveToPosition();
    this.moveCommand = new ManipulatorMoveCommand(this, Integer.MAX_VALUE, tolerance, kP, kI, kD);
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

  /** Move the manipulator to a position in degrees */
  public void moveToPosition(int position) {
    stopCommands();
    moveCommand.setTargetPosition(position);
    moveCommand.schedule();
  }

  public boolean isAtPosition() {
    return Math.abs(getPosition() - getTargetPosition()) < moveCommand.getTolerance();
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

  /**
   * Set the raw speed of the motor in the range -1 to 1
   * <p>
   * Used for manual control
   */
  public void setPower(double percent) {
    stopCommands();
    for (CANSparkMax motor : motors) {
      motor.set(percent);
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
    return Math.abs(getCurrentSpeed() - getTargetSpeed()) < speedCommand.getTolerance();
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
  private void updateEncoders() {
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

  /** Use to invert specific motors {@link #setInverted(boolean)} */
  public void invertSpecificMotors(boolean inverted, int... motorIndexes) {
    for (int index : motorIndexes) {
      motors.get(index).setInverted(inverted);
      motors.get(index).getEncoder().setInverted(inverted);
    }
  }

  /**
   * Move all motors to known position and then reset encoders
   * <p>
   * Do it yourself
   */
  public void home() {
    throw new UnsupportedOperationException("Homing not implemented on: " + this.getClass().getName());
  }

  public void stopCommands() {
    cancelMoveToPosition();
    cancelSpeedCommand();
  }

  public void fullStop() {
    stopCommands();
    stop();
  }
}
