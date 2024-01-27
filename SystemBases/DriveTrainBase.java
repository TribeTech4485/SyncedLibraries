package frc.robot.SyncedLibraries.SystemBases;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class DriveTrainBase extends SubsystemBase {
  private ArrayList<CANSparkMax> leftMotors = new ArrayList<CANSparkMax>(); // define in constructor
  private ArrayList<CANSparkMax> rightMotors = new ArrayList<CANSparkMax>(); // define in constructor
  private ArrayList<CANSparkMax> motors = new ArrayList<CANSparkMax>(); // define in constructor

  // define Speed Controller Groups and Differential Drive for use in drive train
  private CANSparkMax driveMainLeft;
  private CANSparkMax driveMainRight;
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
  public DriveTrainBase(CANSparkMax[] leftDriveMotors, CANSparkMax[] rightDriveMotors, PneumaticsModuleType moduleType,
      int[] GearChangerPorts, boolean kSkipGyro, double maxSpeed, int driveAmpsMax, double drivingRamp,
      double wheelDiameter, double pulsesPerRevolution, boolean disableShifter) {
    System.out.print("Instatntiating drivetrain");
    for (CANSparkMax motor : leftDriveMotors) {
      leftMotors.add(motor);
    }
    for (CANSparkMax motor : rightDriveMotors) {
      rightMotors.add(motor);
    }

    motors.addAll(leftMotors);
    motors.addAll(rightMotors);

    // Sync side motors
    for (CANSparkMax motor : leftMotors) {
      if (motor == leftMotors.get(0)) {
        driveMainLeft = motor;
      } else {
        motor.follow(driveMainLeft);
      }
    }
    for (CANSparkMax motor : rightMotors) {
      if (motor == rightMotors.get(0)) {
        driveMainRight = motor;
      } else {
        motor.follow(driveMainRight);
      }
    }

    // Update motor config
    for (CANSparkMax motor : motors) {
      motor.restoreFactoryDefaults();
      motor.setSmartCurrentLimit(driveAmpsMax);
      motor.setClosedLoopRampRate(drivingRamp);
      motor.setOpenLoopRampRate(drivingRamp);
      motor.setIdleMode(IdleMode.kCoast);
      motor.enableVoltageCompensation(11);
      motor.burnFlash();
    }

    // Invert 1 side of robot so will drive forward
    driveMainLeft.setInverted(true);

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
        m_Gyro = new AHRS(SPI.Port.kMXP);
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
      driveMainLeft.set(rightDrivePercent);
    } else {
      driveMainLeft.stopMotor();
    }
    if (Math.abs(rightDrivePercent) > 0.01) {
      driveMainRight.set(rightDrivePercent);
    } else {
      driveMainRight.stopMotor();
    }
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

  /** Artificial speed limit, 1/3 */
  public void doSlowMode(boolean slow) {
    isSlowMode = slow;
    if (isSlowMode) {
      speedMultiplier = 0.3;
    } else {
      speedMultiplier = maxSpeed;
    }
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
    for (CANSparkMax motor : motors) {
      current += motor.getOutputCurrent();
    }
    return current / motors.size();
  }

  public void setBrakeMode(boolean brake) {
    if (brake) {
      for (CANSparkMax motor : motors) {
        motor.setIdleMode(IdleMode.kBrake);
      }
    } else {
      for (CANSparkMax motor : motors) {
        motor.setIdleMode(IdleMode.kCoast);
      }
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
}