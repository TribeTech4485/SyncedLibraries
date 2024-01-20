package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * To impliment:
 * <p>
 * Create class that extends this class, override all methods besides
 * the set___Command methods, and then run the set___Command methods
 */
public class ManipulatorBase extends SubsystemBase {
  ManipulatorMoveCommand moveCommand;
  ManipulatorSpeedCommand speedCommand;
  double positionMultiplier = 1;
  double speedMultiplier = 1;

  // ===================== Positional Methods ===================== //
  /** Get the known position of the manipulator in degrees */
  public double getPosition() {
    throw new UnsupportedOperationException("Not implemented");
  }

  /**
   * Set the known position of the manipulator in degrees.
   * <p>
   * USE SPARINGLY
   */
  public void _setPosition(double position) {
    throw new UnsupportedOperationException("Not implemented");
  }

  /** Set multiplier to convert from encoder values to degrees on manipulator */
  public void setPositionMultiplier(double multiplier) {
    positionMultiplier = multiplier;
  }

  /** DO NOT RUN WHILE {@link ManipulatorMoveCommand} IS IN USE */
  public void setMoveCommand(double tolerance, double kP, double kI, double kD) {
    cancelMoveToPosition();
    this.moveCommand = new ManipulatorMoveCommand(this, Integer.MAX_VALUE, tolerance, kP, kI, kD);
  }

  /** Get the target position of the manipulator in degrees */
  public double getTargetPosition() {
    if (moveCommand == null) {
      return Integer.MAX_VALUE;
    }
    return moveCommand.getTargetPosition();
  }

  /** Move the manipulator to a position in degrees */
  public void moveToPosition() {
    stopCommands();
    moveCommand.setTargetPosition(getPosition());
    moveCommand.schedule();
  }

  public void cancelMoveToPosition() {
    if (moveCommand != null && moveCommand.isScheduled()) {
      CommandScheduler.getInstance().cancel(moveCommand);
      System.out.println("ManipulatorBase: Cancelling move command");
    }
  }

  // ===================== Raw Speed Methods ===================== //
  // Whenever overriding raw speed methods, make sure to call stopCommands() in the method
  /** Get the raw speed of the motor in the range -1 to 1 */
  public double getPower() {
    throw new UnsupportedOperationException("Not implemented");
  }

  /** Set the raw speed of the motor in the range -1 to 1 
   * <p>
   * Used for manual control
  */
  public void setPower(double power) {
    throw new UnsupportedOperationException("Not implemented");
  }

  // ===================== Speed Methods ===================== //

  /** Set multiplier to convert from encoder values to rpm on manipulator */
  public void setSpeedMultiplier(double multiplier) {
    speedMultiplier = multiplier;
  }

  /** Get the live speed of the motor in rpm */
  public double getCurrentSpeed() {
    throw new UnsupportedOperationException("Not implemented");
  }

  /** Get the target speed of the motor in rpm */
  public double getTargetSpeed() {
    if (speedCommand == null) {
      return Integer.MAX_VALUE;
    }
    return speedCommand.getTargetSpeed();
  }

  /** Set the speed of the motor in rpm */
  public void setSpeed(double speed) {
    stopCommands();
    speedCommand.setTargetSpeed(speed);
    speedCommand.schedule();
  }

  /** DO NOT RUN WHILE {@link ManipulatorSpeedCommand} IS IN USE */
  public void setSpeedCommand(double tolerance, double kP, double kI, double kD) {
    cancelSpeedCommand();
    this.speedCommand = new ManipulatorSpeedCommand(this, Integer.MAX_VALUE, tolerance, kP, kI, kD);
  }

  public void cancelSpeedCommand() {
    if (speedCommand != null && speedCommand.isScheduled()) {
      CommandScheduler.getInstance().cancel(speedCommand);
      System.out.println("ManipulatorBase: Cancelling speed command");
    }
  }

  
  // ===================== Utility Methods ===================== //
  /** Stop all motors
   * <p>
   * DO NOT FORGET TO STOP COMMANDS
   */
  public void stop() {
    throw new UnsupportedOperationException("Not implemented");
  }

  /** Not available on all setups */
  public void setBrakeMode(boolean brakeOnStop) {
    throw new UnsupportedOperationException("Not implemented");
  }

  /** Use to invert all motor functions
   * <p>
   * Don't forget to invert the encoder if needed!
   */
  public void setInverted(boolean inverted) {
    throw new UnsupportedOperationException("Not implemented");
  }

  /** Move all motors to known position and then reset encoders */
  public void home() {
    throw new UnsupportedOperationException("Not implemented");
  }

  public void stopCommands() {
    cancelMoveToPosition();
    cancelSpeedCommand();
  }
}
