package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SyncedLibraries.SystemBases.SpeedManipulatorBase;
import com.revrobotics.spark.SparkMax;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.revrobotics.RelativeEncoder;

public class ManipulatorSpeedCommand extends Command {
  private SpeedManipulatorBase manipulator;
  private AngularVelocity targetSpeed;
  private boolean endOnTarget = false;
  private VelocityFFController[] pidFF;
  private SparkMax[] motors;
  private RelativeEncoder[] encoders;
  public AngularVelocity tolerance = RadiansPerSecond.of(0.1);
  public int onTargetCounterStart = 1;
  private int onTargetCounter = onTargetCounterStart;
  public boolean atSpeed = false;
  protected PIDConfig pidConfig;

  public ManipulatorSpeedCommand(SpeedManipulatorBase manipulator, AngularVelocity speed,
      PIDConfig pidConfig) {
    this.manipulator = manipulator;
    this.targetSpeed = speed;
    this.pidConfig = pidConfig;
    motors = manipulator.getMotors();
    encoders = manipulator.getEncoders();
    pidFF = new VelocityFFController[motors.length];
    for (int i = 0; i < motors.length; i++) {
      pidFF[i] = new VelocityFFController(pidConfig);
    }

    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.stop();
    for (int i = 0; i < motors.length; i++) {
      pidFF[i].reset(manipulator.getCurrentSpeed());
      pidFF[i].setGoal(targetSpeed);
    }
  }

  @Override
  public void execute() {
    for (int i = 0; i < motors.length; i++) {
      Voltage pidVoltage = pidFF[i].calculate(RadiansPerSecond.of(encoders[i].getVelocity()));
      motors[i].set(pidVoltage.magnitude());
    }
    atSpeed = isAtSpeed();
  }

  @Override
  public void end(boolean interrupted) {
    manipulator.stop();
  }

  @Override
  public boolean isFinished() {
    return endOnTarget && atSpeed;
  }

  public void setTargetSpeed(AngularVelocity speed) {
    this.targetSpeed = speed;
  }

  public AngularVelocity getTargetSpeed() {
    return targetSpeed;
  }

  private boolean isAtSpeed() {
    if (currentlyAtSpeed()) {
      onTargetCounter--;
    } else {
      onTargetCounter = onTargetCounterStart;
    }
    return onTargetCounter <= 0;
  }

  private boolean currentlyAtSpeed() {
    for (int i = 0; i < motors.length; i++) {
      if (encoders[i].getVelocity() - targetSpeed.in(RadiansPerSecond) > tolerance.in(RadiansPerSecond) * 1.5) {
        return false;
      }
    }
    return Math.abs(
        manipulator.getCurrentSpeed().minus(targetSpeed).in(RadiansPerSecond)) < tolerance.in(RadiansPerSecond) * 1.5;
  }

  public ManipulatorSpeedCommand setEndOnTarget(boolean endOnTarget) {
    this.endOnTarget = endOnTarget;
    return this;
  }
}
