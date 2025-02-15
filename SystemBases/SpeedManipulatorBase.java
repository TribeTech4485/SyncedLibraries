package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorSpeedCommand;

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
 * 4.) Run the {@link #setSpeedPID(ManipulatorSpeedCommand)} methods
 * in the constructor to set the PID values.
 * <p>
 * 5.) Adjust speed multiplier to be in radians per second
 * <p>
 * You also can use standard {@link SubsystemBase} and {@link ManipulatorBase}
 * methods
 * <p>
 * Add a loop to run all tests to Robot.testInit()
 */
public abstract class SpeedManipulatorBase extends ManipulatorBase {

  /** If null, run the {@link #setSpeedPID(double, double, double, double)} */
  protected ManipulatorSpeedCommand speedCommand;

  /** Get the target speed of the motor */
  public AngularVelocity getTargetSpeed() {
    return speedCommand.getTargetSpeed();
  }

  /** Set the speed of the motor in rpm */
  public void setTargetSpeed(AngularVelocity speed) {
    stopCommand();
    speedCommand.setTargetSpeed(speed);
    speedCommand.schedule();
  }

  public ManipulatorSpeedCommand getSpeedCommand() {
    return speedCommand;
  }

  public boolean isAtSpeed() {
    return speedCommand.atSpeed;
  }

  public void setSpeedPID(double kP, double kI, double kD, AngularVelocity tolerance) {
    cancelSpeedCommand();
    this.speedCommand = new ManipulatorSpeedCommand(this, null, tolerance, kP, kI, kD);
  }

  public void setSpeedPID(ManipulatorSpeedCommand command) {
    cancelSpeedCommand();
    this.speedCommand = command;
  }

  public void setSpeedPID(double kP, double kI, double kD, double kS, double kV, double kA,
      AngularAcceleration maxAccel) {
    cancelSpeedCommand();
    this.speedCommand = new ManipulatorSpeedCommand(this, null, kP, kI, kD, kS, kV, kA, maxAccel);
  }

  public final void cancelSpeedCommand() {
    if (speedCommand != null && speedCommand.isScheduled()) {
      CommandScheduler.getInstance().cancel(speedCommand);
      System.out.println("ManipulatorBase: Cancelling speed command");
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putBoolean(getName() + " At Speed", isAtSpeed());
  }

  @Override
  public void stopCommand() {
    cancelSpeedCommand();
  }
}
