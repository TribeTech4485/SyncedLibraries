package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularAcceleration;
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
  private AngularVelocity tolerance;
  private boolean endOnTarget = false;
  private PIDController[] pid;
  private VelocityFFController[] pidFF;
  private SparkMax[] motors;
  private RelativeEncoder[] encoders;
  public int onTargetCounterStart = 1;
  private int onTargetCounter = onTargetCounterStart;
  public boolean atSpeed = false;

  public ManipulatorSpeedCommand(SpeedManipulatorBase manipulator, AngularVelocity speed, AngularVelocity tolerance,
      double kP, double kI, double kD) {
    this.manipulator = manipulator;
    this.targetSpeed = speed;
    this.tolerance = tolerance;
    motors = manipulator.getMotors();
    encoders = manipulator.getEncoders();
    PIDController[] pidList = new PIDController[motors.length];
    for (int i = 0; i < motors.length; i++) {
      pidList[i] = new PIDController(kP, kI, kD);
      pidList[i].setTolerance(tolerance.in(RadiansPerSecond));
    }
    pid = pidList;

    addRequirements(manipulator);
  }

  public ManipulatorSpeedCommand(SpeedManipulatorBase manipulator, AngularVelocity speed,
      double kP, double kI, double kD, double kS, double kV, double kA, AngularAcceleration maxAccel) {
    this.manipulator = manipulator;
    this.targetSpeed = speed;
    motors = manipulator.getMotors();
    encoders = manipulator.getEncoders();
    VelocityFFController[] pidFFList = new VelocityFFController[motors.length];
    for (int i = 0; i < motors.length; i++) {
      pidFFList[i] = new VelocityFFController(kP, kI, kD, kS, kV, kA, maxAccel);
      pidFFList[i].setTolerance(tolerance);
    }
    pidFF = pidFFList;

    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.stop();
    for (int i = 0; i < motors.length; i++) {
      if (pid != null) {
        pid[i].reset();
        pid[i].setTolerance(tolerance.in(RadiansPerSecond));
        pid[i].setSetpoint(targetSpeed.in(RadiansPerSecond));
      } else {
        pidFF[i].reset(manipulator.getCurrentSpeed());
        pidFF[i].setTolerance(tolerance);
        pidFF[i].setGoal(targetSpeed);
      }
    }
  }

  @Override
  public void execute() {
    for (int i = 0; i < motors.length; i++) {
      if (pid != null) {
        double pidSpeed = pid[i].calculate(encoders[i].getVelocity(), targetSpeed.in(RadiansPerSecond));
        motors[i].set(pidSpeed);
      } else {
        Voltage pidVoltage = pidFF[i].calculate(RadiansPerSecond.of(encoders[i].getVelocity()));
        motors[i].set(pidVoltage.magnitude());
      }
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

  public AngularVelocity getTolerance() {
    return tolerance;
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
