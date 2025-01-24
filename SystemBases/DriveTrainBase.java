package frc.robot.SyncedLibraries.SystemBases;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;

/** USED FOR TANK-DRIVE BASES */
public abstract class DriveTrainBase extends SubsystemBase {
  private LinkedList<SparkMax> leftMotors = new LinkedList<SparkMax>(); // define in constructor
  private LinkedList<SparkMax> rightMotors = new LinkedList<SparkMax>(); // define in constructor
  private LinkedList<SparkMax> motors = new LinkedList<SparkMax>(); // define in constructor

  // define Speed Controller Groups and Differential Drive for use in drive train
  private SparkMax driveMainLeft;
  private SparkMax driveMainRight;
  private DifferentialDrive differentialDrive;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  // navX Gyro on RoboRIO 2.0
  private AHRS m_Gyro;

  private int counter = 0; // for limiting display
  private double speedMultiplier;
  private double maxSpeed;
  private double WHEEL_DIAMETER;
  private double PULSES_PER_REVOLUTION;
  private boolean isHighGear = false;
  private boolean isSlowMode = false;

  private DoubleSolenoid gearChanger;

  /**
   * Constructor for DriveTrainBase
   *
   * @param leftDriveMotors     Array of left drive motors
   * @param rightDriveMotors    Array of right drive motors
   * @param moduleType          Pneumatics module type
   * @param GearChangerPorts    Array of 2 ports for the gear changer, or null if
   *                            empty
   * @param kSkipGyro           Whether to skip the gyro
   * @param maxSpeed            Maximum speed of the robot
   * @param driveAmpsMax        Maximum amps for the drive motors
   * @param drivingRamp         Ramp rate for the drive motors
   * @param wheelDiameter       Diameter of the wheels
   * @param pulsesPerRevolution Pulses per revolution of the encoders
   * @param disableShifter      Whether to disable the shifter
   */
  public DriveTrainBase(SparkMax[] leftDriveMotors, SparkMax[] rightDriveMotors, PneumaticsModuleType moduleType,
      int[] GearChangerPorts, boolean kSkipGyro, double maxSpeed, int driveAmpsMax, double drivingRamp,
      double wheelDiameter, double pulsesPerRevolution, boolean disableShifter) {
    System.out.print("Instatntiating drivetrain");
    for (SparkMax motor : leftDriveMotors) {
      leftMotors.add(motor);
    }
    for (SparkMax motor : rightDriveMotors) {
      rightMotors.add(motor);
    }

    motors.addAll(leftMotors);
    motors.addAll(rightMotors);

    // Update motor config
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig()
          .smartCurrentLimit(driveAmpsMax)
          .closedLoopRampRate(drivingRamp)
          .openLoopRampRate(drivingRamp)
          .idleMode(IdleMode.kCoast)
          .inverted(false),
          ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Sync side motors
    for (SparkMax motor : leftDriveMotors) {
      motor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters,
          PersistMode.kPersistParameters);
      if (motor == leftMotors.get(0)) {
        driveMainLeft = motor;
      } else {
        // motor.follow(driveMainLeft);
      }
    }
    for (SparkMax motor : rightDriveMotors) {
      motor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters,
          PersistMode.kPersistParameters);
      if (motor == rightMotors.get(0)) {
        driveMainRight = motor;
      } else {
        // motor.follow(driveMainRight);
      }
    }

    // Invert 1 side of robot so will drive forward
    // driveMainLeft.setInverted(true);

    differentialDrive = new DifferentialDrive(driveMainLeft, driveMainRight);
    differentialDrive.setSafetyEnabled(false);

    m_leftEncoder = driveMainLeft.getEncoder();
    m_rightEncoder = driveMainRight.getEncoder();

    // Initialize the solenoids
    if (GearChangerPorts.length == 2 && !disableShifter) {
      gearChanger = new DoubleSolenoid(5, moduleType, GearChangerPorts[0], GearChangerPorts[1]);
      gearChanger.set(DoubleSolenoid.Value.kOff);
    } else {
      gearChanger = null;
    }

    if (kSkipGyro) {
      m_Gyro = null;
    } else {
      // navX-MXP Gyro instantiation
      try {
        m_Gyro = new AHRS(NavXComType.kMXP_SPI);
      } catch (RuntimeException ex) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
      while (m_Gyro.isCalibrating()) {
        try {
          Thread.sleep(500);
        } catch (Exception e) {
          System.out.println(e);
        } // sleep in milliseconds
        System.out.print("**gyro isCalibrating . . .");
      }
      // SmartDashboard.putBoolean("gyro connected", m_Gyro.isConnected());
      System.out.print("gyro connected " + m_Gyro.isConnected());
    }
    System.out.println(" ... Done");

    this.speedMultiplier = maxSpeed;
    this.maxSpeed = maxSpeed;
    this.WHEEL_DIAMETER = wheelDiameter;
    this.PULSES_PER_REVOLUTION = pulsesPerRevolution;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Drive Speed", getLeftSpeed());
    SmartDashboard.putNumber("Right Drive Speed", getRightSpeed());
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left  Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void doTankDrive(double leftDrivePercent, double rightDrivePercent) {
    leftDrivePercent *= speedMultiplier;
    rightDrivePercent *= speedMultiplier;

    if (counter++ % 100 == 0) {
      System.out.println("**driveTrain power L/R: " + leftDrivePercent + " | " + rightDrivePercent);
    }
    if (Math.abs(leftDrivePercent) > 0.01) {
      for (SparkMax motor : leftMotors) {
        motor.set(leftDrivePercent);
      }
      // driveMainLeft.set(leftDrivePercent);
    } else {
      for (SparkMax motor : leftMotors) {
        motor.stopMotor();
      }
      // driveMainLeft.stopMotor();
    }
    if (Math.abs(rightDrivePercent) > 0.01) {
      for (SparkMax motor : rightMotors) {
        motor.set(-rightDrivePercent);
      }
      // driveMainRight.set(rightDrivePercent);
    } else {
      for (SparkMax motor : rightMotors) {
        motor.stopMotor();
      }
      // driveMainRight.stopMotor();
    }
    // differentialDrive.tankDrive(leftDrivePercent, rightDrivePercent);
  }

  /**
   * Arcade style driving for the DriveTrain.
   *
   * @param speed    Speed in range [-1,1]
   * @param rotation Rotation in range [-1,1]
   */
  public void doArcadeDrive(double speed, double rotation) {
    // if (counter++ % 100 == 0) { System.out.println("**arcadeDrive power
    // speed/rotation: " + speed+"-"rotation); }
    differentialDrive.arcadeDrive(speed * speedMultiplier, rotation);
  }

  public double getHeadingAngle() {
    return m_Gyro.getAngle();
  }

  public double getYaw() {
    return m_Gyro.getYaw();
  }

  public void resetGyro() {
    m_Gyro.reset();
    m_Gyro.zeroYaw();
  }

  // public void resetEncoder() {
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftEncoderCount() {
    return m_leftEncoder.getPosition();
    // return 0;
  }

  public double getRightEncoderCount() {
    return m_rightEncoder.getPosition();
    // return 0;
  }

  public double getLeftDistanceInch() {
    return Math.PI * WHEEL_DIAMETER * (getLeftEncoderCount() /
        PULSES_PER_REVOLUTION);
    // return 0.0;
  }

  public double getRightDistanceInch() {
    return Math.PI * WHEEL_DIAMETER * (getRightEncoderCount() /
        PULSES_PER_REVOLUTION);
    // return 0.0;
  }

  public double getAveDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
    // return 0.0;
  }

  // Function to set the solenoids
  public void doHighGear(boolean fast) {
    if (gearChanger == null) {
      System.out.println("No gear changer, and doHighGear called");
      return;
    }
    isHighGear = fast;
    if (isHighGear) {
      gearChanger.set(DoubleSolenoid.Value.kReverse);
      System.out.println("Gear shifter set to High Speed Mode");
    } else {
      gearChanger.set(DoubleSolenoid.Value.kForward);
      System.out.println("Gear shifter set to High Torque Mode");
    }
  }

  public void doSlowMode(boolean slow) {
    isSlowMode = slow;
    if (isSlowMode) {
      speedMultiplier = 0.2;
    } else {
      speedMultiplier = maxSpeed;
    }
  }

  /** AKA set speed multiplier */
  public void doSlowMode(double speed) {
    speedMultiplier = Math.max(Math.min(speed, maxSpeed), -maxSpeed);
  }

  public void stop() {
    System.out.println("Drivetrain stop");
    doTankDrive(0.0, 0.0);
  }

  public double getLeftSpeed() {
    return driveMainLeft.get();
  }

  public double getRightSpeed() {
    return driveMainRight.get();
  }

  public double getAveCurrent() {
    double current = 0;
    for (SparkMax motor : motors) {
      current += motor.getOutputCurrent();
    }
    return current / motors.size();
  }

  public void setBrakeMode(boolean brake) {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast),
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  public SparkMax[] getAllMotors() {
    return motors.toArray(new SparkMax[0]);
  }

  public void invertAll() {
    for (SparkMax motor : motors) {
      motor.configure(new SparkMaxConfig().inverted(!motor.configAccessor.getInverted()),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
  }

  public void resetAll() {
    System.out.println("Resetting DriveTrain");
    stop();
    resetEncoders();
    resetGyro();
    doHighGear(false);
    doSlowMode(false);
    // setBrakeMode(true);
  }

  public AHRS getGyro() {
    return m_Gyro;
  }

  public void ESTOP() {
    setBrakeMode(true);
    stop();
  }

  public void onDisable() {
    stop();
    for (SparkMax motor : motors) {
      // Save motor configurations
      motor.configure(new SparkMaxConfig(),
          ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
}