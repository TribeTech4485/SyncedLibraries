package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class VelocityFFController implements Sendable, AutoCloseable {
  public ProfiledPIDController pidController;
  SimpleMotorFeedforward ffController;
  double kS;
  double kV;

  public VelocityFFController(double kP, double kI, double kD, double kS, double kV, double kA, double maxVelocity,
      double maxAccel) {
    pidController = new ProfiledPIDController(kP, kI, kD,
        new TrapezoidProfile.Constraints(maxAccel, maxAccel * 10000));
    ffController = new SimpleMotorFeedforward(0, kA);
    this.kS = kS;
    this.kV = kV;
  }

  public void setGoal(double goal) {
    pidController.setGoal(goal);
  }

  public double calculate(double currentVelocity) {
    return pidController.calculate(currentVelocity) +
        ffController.calculate(pidController.getSetpoint().velocity)
        + kV * pidController.getSetpoint().position
        + kS * Math.signum(0 - pidController.getSetpoint().position);
  }

  public State getGoal() {
    return pidController.getGoal();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty("p", pidController::getP, pidController::setP);
    builder.addDoubleProperty("i", pidController::getI, pidController::setI);
    builder.addDoubleProperty("d", pidController::getD, pidController::setD);
    builder.addDoubleProperty("f", () -> kS, (value) -> kS = value);
    builder.addDoubleProperty("v", () -> kV, (value) -> kV = value);
    builder.addDoubleProperty("a", ffController::getKv,
        (value) -> ffController = new SimpleMotorFeedforward(0, value));
    builder.addDoubleProperty("setpoint", () -> pidController.getSetpoint().position, pidController::setGoal);
    builder.addDoubleProperty("velocity", () -> pidController.getSetpoint().position, null);
    builder.addDoubleProperty("error", pidController::getPositionError, null);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }
}
