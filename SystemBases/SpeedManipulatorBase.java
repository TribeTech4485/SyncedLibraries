package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorSpeedCommand;
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
  protected final PIDConfig pidConfig;

  /** If null, run the {@link #setSpeedPID(double, double, double, double)} */
  protected final ManipulatorSpeedCommand speedCommand;

  public SpeedManipulatorBase(PIDConfig pidConfig) {
    this.pidConfig = pidConfig;
    speedCommand = new ManipulatorSpeedCommand(this, RotationsPerSecond.of(0), pidConfig);
  }

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

  public PIDConfig getPIDConfig() {
    return pidConfig;
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
