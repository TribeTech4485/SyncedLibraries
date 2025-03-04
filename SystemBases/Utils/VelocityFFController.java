package frc.robot.SyncedLibraries.SystemBases.Utils;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class VelocityFFController implements Sendable, AutoCloseable {
  public ProfiledPIDController pidController;
  public final PIDConfig pidConfig;
  /**
   * Should be RadiansPerSecond, but may be rotations per second ie. for driving
   */
  protected AngularVelocityUnit angularVelocityUnit = RadiansPerSecond;

  public VelocityFFController(PIDConfig pidConfig) {
    pidController = new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(0, 0));
    this.pidConfig = pidConfig;
    synchronizePIDSettings();
  }

  public VelocityFFController(PIDConfig pidConfig, AngularVelocityUnit angularVelocityUnit) {
    pidController = new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(0, 0));
    this.pidConfig = pidConfig;
    this.angularVelocityUnit = angularVelocityUnit;
    synchronizePIDSettings();
  }

  public void setGoal(AngularVelocity goal) {
    pidController.setGoal(goal.in(angularVelocityUnit));
  }

  public Voltage calculate(AngularVelocity currentVelocity) {
    return Volts.of(pidController.calculate(currentVelocity.in(angularVelocityUnit)) +
        pidConfig.A * (pidController.getSetpoint().velocity)
        + pidConfig.V * pidController.getSetpoint().position
        + pidConfig.S * Math.signum(0 - pidController.getSetpoint().position));
  }

  public AngularVelocity getGoal() {
    return AngularVelocity.ofBaseUnits(pidController.getGoal().position, angularVelocityUnit);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty("p", () -> pidConfig.P, (value) -> pidConfig.P = value);
    builder.addDoubleProperty("i", () -> pidConfig.I, (value) -> pidConfig.I = value);
    builder.addDoubleProperty("d", () -> pidConfig.D, (value) -> pidConfig.D = value);
    builder.addDoubleProperty("s", () -> pidConfig.S, (value) -> pidConfig.S = value);
    builder.addDoubleProperty("v", () -> pidConfig.V, (value) -> pidConfig.V = value);
    builder.addDoubleProperty("a", () -> pidConfig.A, (value) -> pidConfig.A = value);
    builder.addDoubleProperty("setpoint", () -> pidController.getSetpoint().position, pidController::setGoal);
    builder.addDoubleProperty("velocity", () -> pidController.getSetpoint().position, null);
    builder.addDoubleProperty("error", pidController::getPositionError, null);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  public void setTolerance(AngularVelocity tolerance) {
    pidController.setTolerance(tolerance.in(angularVelocityUnit));
  }

  public void reset(AngularVelocity initialState) {
    pidController.reset(initialState.in(angularVelocityUnit));
  }

  protected void synchronizePIDSettings() {
    pidConfig.applyTo(pidController, angularVelocityUnit);
  }

  public ProfiledPIDController getPIDController() {
    return pidController;
  }

  public double getSetpoint() {
    return pidController.getSetpoint().position;
  }
}
