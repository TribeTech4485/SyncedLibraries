package frc.robot.SyncedLibraries.SystemBases.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class to store PID values to easily transfer between classes
 * <p>
 * Use the set() methods to set the values, and the applyTo() methods to apply
 * them to
 * the PIDController or ProfiledPIDControllers periodically.
 * <p>
 * Use {@link SmartDashboard#putData(String key, Sendable data)} to send to
 * SmartDashboard for easy tuning. (Does not work with Shuffleboard)
 * <p>
 * All units should be in meters or radians!!
 */
public class PIDConfig implements Sendable {
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#introduction-to-dc-motor-feedforward
  public double P = 0;
  public double I = 0;
  public double D = 0;
  public double S = 0;
  public double V = 0;
  public double A = 0;
  public double G = 0;

  public Distance linearTolerance = Inches.of(0.5);
  public Angle angleTolerance = Degrees.of(1);

  public LinearVelocity maxLinearVelocity = MetersPerSecond.zero();
  public LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.zero();

  public AngularVelocity maxAngularVelocity = RadiansPerSecond.zero();
  public AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.zero();

  public FeedForwardType feedForwardType = FeedForwardType.SimpleMotor;

  /** Modifies in place and returns itself */
  public PIDConfig set(double P, double I, double D) {
    this.P = P;
    this.I = I;
    this.D = D;
    return this;
  }

  /** Modifies in place and returns itself */
  public PIDConfig set(double P, double I, double D, double S, double V, double A, double G,
      LinearVelocity maxVelocity, LinearAcceleration maxAcceleration, FeedForwardType feedForwardType) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.S = S;
    this.V = V;
    this.A = A;
    this.G = G;
    this.maxLinearVelocity = maxVelocity;
    this.maxLinearAcceleration = maxAcceleration;
    this.feedForwardType = feedForwardType;
    return this;
  }

  /** Modifies in place and returns itself */
  public PIDConfig set(double P, double I, double D, double S, double V, double A,
      LinearVelocity maxVelocity, LinearAcceleration maxAcceleration, FeedForwardType feedForwardType) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.S = S;
    this.V = V;
    this.A = A;
    this.maxLinearVelocity = maxVelocity;
    this.maxLinearAcceleration = maxAcceleration;
    this.feedForwardType = feedForwardType;
    return this;
  }

  /** Modifies in place and returns itself */
  public PIDConfig set(double P, double I, double D, double S, double V, double A, double G,
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration, FeedForwardType feedForwardType) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.S = S;
    this.V = V;
    this.A = A;
    this.G = G;
    this.maxAngularVelocity = maxVelocity;
    this.maxAngularAcceleration = maxAcceleration;
    this.feedForwardType = feedForwardType;
    return this;
  }

  /** Modifies in place and returns itself */
  public PIDConfig set(double P, double I, double D, double S, double V, double A,
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration, FeedForwardType feedForwardType) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.S = S;
    this.V = V;
    this.A = A;
    this.maxAngularVelocity = maxVelocity;
    this.maxAngularAcceleration = maxAcceleration;
    this.feedForwardType = feedForwardType;
    return this;
  }

  /** Modifies in place and returns itself */
  public PIDConfig set(double P, double I, double D,
      LinearVelocity maxLinearVelocity, LinearAcceleration maxLinearAcceleration) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxLinearAcceleration = maxLinearAcceleration;
    return this;
  }

  /** Modifies in place and returns itself */
  public PIDConfig set(double P, double I, double D,
      AngularVelocity maxAngularVelocity, AngularAcceleration maxAngularAcceleration) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.maxAngularVelocity = maxAngularVelocity;
    this.maxAngularAcceleration = maxAngularAcceleration;
    return this;
  }

  /** Modifies in place and returns itself */
  public PIDConfig set(double P, double I, double D, double S, double V, double A, double G,
      LinearVelocity maxLinearVelocity, LinearAcceleration maxLinearAcceleration,
      AngularVelocity maxAngularVelocity, AngularAcceleration maxAngularAcceleration, FeedForwardType feedForwardType) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.S = S;
    this.V = V;
    this.A = A;
    this.G = G;
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxLinearAcceleration = maxLinearAcceleration;
    this.maxAngularVelocity = maxAngularVelocity;
    this.maxAngularAcceleration = maxAngularAcceleration;
    this.feedForwardType = feedForwardType;
    return this;
  }

  /** Modifies in place to copy the input and returns itself */
  public PIDConfig set(PIDConfig config) {
    this.P = config.P;
    this.I = config.I;
    this.D = config.D;
    this.S = config.S;
    this.V = config.V;
    this.A = config.A;
    this.G = config.G;
    this.maxLinearVelocity = config.maxLinearVelocity;
    this.maxLinearAcceleration = config.maxLinearAcceleration;
    this.maxAngularVelocity = config.maxAngularVelocity;
    this.maxAngularAcceleration = config.maxAngularAcceleration;
    this.feedForwardType = config.feedForwardType;
    return this;
  }

  public PIDConfig setTolerance(Distance input) {
    this.linearTolerance = input;
    return this;
  }

  public PIDConfig setTolerance(Angle input) {
    this.angleTolerance = input;
    return this;
  }

  /** Modifies in place and returns itself */
  public PIDConfig zero() {
    P = 0d;
    I = 0d;
    D = 0d;
    S = 0d;
    V = 0d;
    A = 0d;
    G = 0d;
    maxLinearVelocity = MetersPerSecond.zero();
    maxLinearAcceleration = MetersPerSecondPerSecond.zero();
    maxAngularVelocity = RadiansPerSecond.zero();
    maxAngularAcceleration = RadiansPerSecondPerSecond.zero();
    feedForwardType = FeedForwardType.SimpleMotor;
    return this;
  }

  public boolean equals(PIDConfig config) {
    return P == config.P && I == config.I && D == config.D && S == config.S
        && V == config.V && A == config.A && G == config.G
        && maxLinearVelocity.equals(config.maxLinearVelocity)
        && maxLinearAcceleration.equals(config.maxLinearAcceleration)
        && maxAngularVelocity.equals(config.maxAngularVelocity)
        && maxAngularAcceleration.equals(config.maxAngularAcceleration)
        && feedForwardType == config.feedForwardType;
  }

  public PIDConfig clone() {
    return new PIDConfig().set(this);
  }

  /** Returns true if any of the feedforward values are not zero */
  public boolean hasFF() {
    return S != 0 || V != 0 || A != 0 || G != 0 ||
        !maxLinearVelocity.equals(MetersPerSecond.zero()) ||
        !maxLinearAcceleration.equals(MetersPerSecondPerSecond.zero()) ||
        !maxAngularVelocity.equals(RadiansPerSecond.zero()) ||
        !maxAngularAcceleration.equals(RadiansPerSecondPerSecond.zero());
  }

  /** Applies the PID values to the PIDController */
  public void applyTo(PIDController controller) {
    if (P != controller.getP()) {
      controller.setP(P);
    }
    if (I != controller.getI()) {
      controller.setI(I);
    }
    if (D != controller.getD()) {
      controller.setD(D);
    }
  }

  /** Applies the PID values to the ProfiledPIDController */
  public void applyTo(ProfiledPIDController controller, Unit velocityUnit) {
    if (P != controller.getP()) {
      controller.setP(P);
    }
    if (I != controller.getI()) {
      controller.setI(I);
    }
    if (D != controller.getD()) {
      controller.setD(D);
    }

    if (velocityUnit instanceof LinearVelocityUnit) {
      LinearVelocityUnit linearVelocityUnits = (LinearVelocityUnit) velocityUnit;
      controller.setConstraints(new TrapezoidProfile.Constraints(
          maxLinearVelocity.in(linearVelocityUnits), maxLinearAcceleration.in(linearVelocityUnits.per(Second))));
    } else if (velocityUnit instanceof AngularVelocityUnit) {
      AngularVelocityUnit angularVelocityUnits = (AngularVelocityUnit) velocityUnit;
      controller.setConstraints(new TrapezoidProfile.Constraints(
          maxAngularVelocity.in(angularVelocityUnits), maxAngularAcceleration.in(angularVelocityUnits.per(Second))));
    } else {
      DriverStation.reportError("Invalid velocity unit: " + velocityUnit, true);
    }
  }

  public enum FeedForwardType {
    SimpleMotor,
    Elevator,
    Arm
  }

  /** Sanitize user inputs */
  private FeedForwardType stringToFeedForwardType(String type) {
    try {
      return FeedForwardType.valueOf(type);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError(
          "Invalid FeedForwardType inputed in SmartDashboard: " + type + ", defaulting to SimpleMotor", false);
      return FeedForwardType.SimpleMotor;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("P", () -> P, (value) -> P = value);
    builder.addDoubleProperty("I", () -> I, (value) -> I = value);
    builder.addDoubleProperty("D", () -> D, (value) -> D = value);
    builder.addDoubleProperty("S", () -> S, (value) -> S = value);
    builder.addDoubleProperty("V", () -> V, (value) -> V = value);
    builder.addDoubleProperty("A", () -> A, (value) -> A = value);
    builder.addDoubleProperty("G", () -> G, (value) -> G = value);
    builder.addDoubleProperty("maxLinearVelocity m/s", () -> maxLinearVelocity.in(MetersPerSecond),
        (value) -> maxLinearVelocity = MetersPerSecond.of(value));
    builder.addDoubleProperty("maxLinearAcceleration m/s^2", () -> maxLinearAcceleration.in(MetersPerSecondPerSecond),
        (value) -> maxLinearAcceleration = MetersPerSecondPerSecond.of(value));
    builder.addDoubleProperty("maxAngularVelocity rad/s", () -> maxAngularVelocity.in(RadiansPerSecond),
        (value) -> maxAngularVelocity = RadiansPerSecond.of(value));
    builder.addStringProperty("FeedForward type",
        () -> feedForwardType.toString(), (value) -> stringToFeedForwardType(value));
    builder.addDoubleProperty("maxAngularAcceleration rad/s^2",
        () -> maxAngularAcceleration.in(RadiansPerSecondPerSecond),
        (value) -> maxAngularAcceleration = RadiansPerSecondPerSecond.of(value));
  }
}