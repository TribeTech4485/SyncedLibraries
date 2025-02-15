package frc.robot.SyncedLibraries.SystemBases.Utils;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class VelocityFFController implements Sendable, AutoCloseable {
  public ProfiledPIDController pidController;
  SimpleMotorFeedforward ffController;
  double kS;
  double kV;

  public VelocityFFController(double kP, double kI, double kD, double kS, double kV, double kA,
      AngularAcceleration maxAccel) {
    pidController = new ProfiledPIDController(kP, kI, kD,
        new TrapezoidProfile.Constraints(maxAccel.in(RadiansPerSecondPerSecond),
            maxAccel.times(10000).in(RadiansPerSecondPerSecond)));
    ffController = new SimpleMotorFeedforward(0, kA);
    this.kS = kS;
    this.kV = kV;
  }

  public void setGoal(AngularVelocity goal) {
    pidController.setGoal(goal.in(RadiansPerSecond));
  }

  public Voltage calculate(AngularVelocity currentVelocity) {
    return Volts.of(pidController.calculate(currentVelocity.in(RadiansPerSecond)) +
        ffController.calculate(pidController.getSetpoint().velocity)
        + kV * pidController.getSetpoint().position
        + kS * Math.signum(0 - pidController.getSetpoint().position));
  }

  public AngularVelocity getGoal() {
    return AngularVelocity.ofBaseUnits(pidController.getGoal().position, RadiansPerSecond);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty("p", pidController::getP, pidController::setP);
    builder.addDoubleProperty("i", pidController::getI, pidController::setI);
    builder.addDoubleProperty("d", pidController::getD, pidController::setD);
    builder.addDoubleProperty("s", () -> kS, (value) -> kS = value);
    builder.addDoubleProperty("v", () -> kV, (value) -> kV = value);
    builder.addDoubleProperty("a", this::getKa,
        (value) -> ffController = new SimpleMotorFeedforward(0, value));
    builder.addDoubleProperty("setpoint", () -> pidController.getSetpoint().position, pidController::setGoal);
    builder.addDoubleProperty("velocity", () -> pidController.getSetpoint().position, null);
    builder.addDoubleProperty("error", pidController::getPositionError, null);
  }

  private double getKa() {
    return ffController.getKv();
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  public void setTolerance(AngularVelocity tolerance) {
    pidController.setTolerance(tolerance.in(RadiansPerSecond));
  }

  public void reset(AngularVelocity initialState) {
    pidController.reset(initialState.in(RadiansPerSecond));
  }
}
