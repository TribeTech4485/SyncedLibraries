package frc.robot.SyncedLibraries.SystemBases;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

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
 * 4.) Adjust speed and position multipliers if needed
 * <p>
 * You also can use standard {@link SubsystemBase} methods
 * <p>
 * Add a loop to run all tests to Robot.testInit()
 */
public abstract class ManipulatorBase extends Estopable {
  /**
   * The maximum current that the breaker can handle
   * <p>
   * Can be exceeded minimally in extreme and brief cases
   */
  protected int breakerMaxAmps = 20;
  // Long-term solution, make a motor object that sends values asynchronously to
  // prevent hanging of main thread of many motor sys-calls
  protected ArrayList<SparkMax> motors = new ArrayList<SparkMax>();
  protected ArrayList<RelativeEncoder> encoders = new ArrayList<RelativeEncoder>();
  /**
   * Custom sensor for general use
   * <p>
   * Used for things like limit switches
   */
  protected BooleanSupplier customSensor = () -> false;

  /**
   * The voltage that the motors will be disabled at
   * <p>
   * Set to 0 to disable
   */
  private static double overVoltageProtection = 18;
  private boolean isDisabled = false;

  // ===================== Raw Speed Methods ===================== //
  /** Get the raw speed of the motor in the range -1 to 1 */
  public double getAvePower() {
    double average = 0;
    for (SparkMax motor : motors) {
      average += motor.get();
    }
    return average / motors.size();
  }

  /** Same as {@link #getAvePower()} but ABSed before averaging */
  public double getAbsPower() {
    double average = 0;
    for (SparkMax motor : motors) {
      average += Math.abs(motor.get());
    }
    return average / motors.size();
  }

  /**
   * Set the raw speed of the motor in the range -1 to 1
   * <p>
   * Used for manual control
   */
  public void setPower(double percent, boolean stopCommands) {
    if (stopCommands) {
      stopCommand();
    }
    for (SparkMax motor : motors) {
      if (isDisabled) {
        return;
      }
      motor.set(percent);
    }
  }

  /**
   * Set the raw power of the motor in the range -1 to 1
   * <p>
   * Used for manual control
   * <p>
   * <b>STOPS COMMANDS</b>
   */
  public void setPower(double percent) {
    setPower(percent, true);
  }

  /** Get the average voltage of the motors */
  public double getAveVoltage() {
    double average = 0;
    for (SparkMax motor : motors) {
      average += motor.getBusVoltage() * motor.get();
    }
    return average / motors.size();
  }

  /** Same as {@link #getAveVoltage()} but ABSed before averaging */
  public double getAbsVoltage() {
    double average = 0;
    for (SparkMax motor : motors) {
      average += Math.abs(motor.getBusVoltage() * motor.get());
    }
    return average / motors.size();
  }

  /**
   * Set the raw speed of the motor in the range -1 to 1
   * <p>
   * Used for manual control
   * @param voltage Uses the unit library to prevent improper usage
   */
  public void setVoltage(Voltage voltage, boolean stopCommands) {
    if (stopCommands) {
      stopCommand();
    }

    // double line = motors.get(0).getBusVoltage();
    // voltage = Math.min(line, Math.max(-line, voltage));
    for (SparkMax motor : motors) {
      if (isDisabled) {
        return;
      }
      motor.setVoltage(voltage);
    }
  }

  // ===================== Encoder Methods ===================== //

  /**
   * Set multiplier to convert from encoder values to radians/s or m/s
   * on the manipulator
   * <p>
   * Warning: you'll likely need to divide by 50 due to 50 periodics/sec
   */
  public void setSpeedMultiplier(double multiplier) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().apply(new EncoderConfig().velocityConversionFactor(multiplier)),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  /** Get the live speed of the motor */
  public AngularVelocity getCurrentSpeed() {
    double average = 0;
    for (RelativeEncoder encoder : encoders) {
      average += encoder.getVelocity();
    }
    average /= encoders.size();
    return RadiansPerSecond.of(average);
  }

  /** Set multiplier to convert from encoder values to meters on manipulator */
  public void setPositionMultiplier(double multiplier) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(multiplier)),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  /** Sets both speed and postition multipliers */
  public void setEncoderMultiplier(double multiplier) {
    setPositionMultiplier(multiplier);
    setSpeedMultiplier(multiplier / 50); // 50 periodics/sec
  }

  // ===================== Utility Methods ===================== //

  /** Adds all the encoders from the motors into the encoders list */
  protected void updateEncoders() {
    encoders.clear();
    for (SparkMax motor : motors) {
      encoders.add(motor.getEncoder());
    }
  }

  /** Do I really need to explain this? */
  public void addMotors(SparkMax... motors) {
    for (SparkMax motor : motors) {
      this.motors.add(motor);
    }
    updateEncoders();
  }

  /**
   * In order of when motors are added, likely in the constructer of the child
   * class
   */
  public SparkMax getMotor(int index) {
    return motors.get(index);
  }

  public RelativeEncoder getEncoder(int index) {
    return encoders.get(index);
  }

  public SparkMax[] getMotors() {
    return motors.toArray(new SparkMax[0]);
  }

  public RelativeEncoder[] getEncoders() {
    return encoders.toArray(new RelativeEncoder[0]);
  }

  /**
   * Stop all motors
   * <p>
   * DOES NOT STOP COMMANDS
   */
  public void stop() {
    for (SparkMax motor : motors) {
      motor.stopMotor();
    }
  }

  /** Reset all motors to factory settings */
  public void resetMotors() {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /** Not reasonable on all setups */
  public void setBrakeMode(boolean brakeOnStop) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().idleMode(brakeOnStop ? IdleMode.kBrake : IdleMode.kCoast),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  /** Use to invert all motor functions, including encoders */
  public void setInverted(boolean inverted) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().inverted(inverted)
          .apply(new EncoderConfig().inverted(inverted)), ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
  }

  /** Inputting -1 sets to the maxBreakerAmps */
  public void setCurrentLimit(int limit) {
    if (limit == -1) {
      setCurrentLimit(breakerMaxAmps);
      return;
    }

    if (limit > breakerMaxAmps) {
      DriverStation.reportWarning(
          "ManipulatorBase: Current limit out of bounds" + limit + " > " + breakerMaxAmps, true);
      if (limit > 1.25 * breakerMaxAmps) {
        limit = (int) (1.25 * breakerMaxAmps);
      }
    }
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().smartCurrentLimit(limit), ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
  }

  /** To prevent setting to draw too many amps */
  public void setBreakerMaxAmps(int amps) {
    breakerMaxAmps = amps;
  }

  /**
   * Outdated way of controlling acceleration,
   * 1 means one second to go from 0-100%
   */
  public void setRampRate(double rate) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig()
          .closedLoopRampRate(rate)
          .openLoopRampRate(rate),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  /** Use to invert specific motors, se also {@link #setInverted(boolean)} */
  public void invertSpecificMotors(boolean inverted, int... motorIndexes) {
    for (int index : motorIndexes) {
      motors.get(index).configure(new SparkMaxConfig().inverted(inverted),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      // motors.get(index).getEncoder().setInverted(inverted);
    }
  }

  /** Is power going to any motors */
  public boolean isRunning() {
    return getAbsPower() != 0;
  }

  public boolean isSpinning() {
    return getCurrentSpeed().in(RadiansPerSecond) >= 0.01;
  }

  /**
   * Move all motors to known position and then reset encoders
   * <p>
   * Do it yourself
   */
  public Command home() {
    return new PrintCommand("Homing not implemented on subsystem " + getName() + "... Continuing");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + " Speed (rpm)", getCurrentSpeed().in(RPM));
    SmartDashboard.putNumber(getName() + " Power", getAvePower());

    if (overVoltageProtection != 0) {
      if (getMotor(0).getBusVoltage() > overVoltageProtection) {
        if (!isDisabled) {
          DriverStation.reportError("Manipulators over voltage protection enabled", false);
          for (SparkMax motor : motors) {
            motor.disable();
          }
          isDisabled = true;
        }
      } else {
        isDisabled = false;
      }
    }
  }

  /**
   * When put into test mode, this command will run
   * <p>
   * Used to run all systems to show they work
   */
  public abstract Command test();

  public static ManipulatorBase[] getAllManipulators() {
    Estopable[] estopables = Estopable.getAllEstopables();
    LinkedList<ManipulatorBase> manipulators = new LinkedList<ManipulatorBase>();
    for (Estopable estopable : estopables) {
      if (estopable instanceof ManipulatorBase) {
        manipulators.add((ManipulatorBase) estopable);
      }
    }
    return manipulators.toArray(new ManipulatorBase[0]);
  }

  /**
   * Runs when the robot is disabled, base method saves motor config
   */
  @Override
  public void onDisable() {
    // Save motor config
    boolean maybe = true;
    if (maybe) {
      return; // ?
    }
    persistMotorConfig();
  }

  public BooleanSupplier getCustomSensor() {
    return customSensor;
  }

  protected void persistMotorConfig() {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig(), ResetMode.kNoResetSafeParameters,
          PersistMode.kPersistParameters);
    }
  }

  /**
   * Stops the speed/position command
   */
  public void stopCommand() {
    // Overridden in abstract subclasses
    System.out.println("ManipulatorBase" + getName() + " stopCommand() not implemented");
  }

  public void fullStop() {
    stop();
    stopCommand();
  }

  /**
   * Runs whenever a manipulatorbase is created, used to ensure that a reasonable
   * current limit is in place
   */
  public ManipulatorBase() {
    // Make sure the amp limit is reasonable
    for (SparkMax motor : motors) {
      if (motor.configAccessor.getSmartCurrentLimit() > breakerMaxAmps) {
        motor.configure(new SparkMaxConfig().smartCurrentLimit(breakerMaxAmps), ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
      }
    }
  }
}