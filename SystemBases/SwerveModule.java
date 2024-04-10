package frc.robot.SyncedLibraries.SystemBases;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;
  private RelativeEncoder turnEncoder;
  private RelativeEncoder driveEncoder;
  private CANcoder absoluteEncoder;
  private SparkPIDController turnPID;
  private double targetTurnPosition = 0;
  private double targetPower = 0;
  private int moduleNumber;

  /**
   * Constructs a SwerveModule object with the specified parameters.
   *
   * @param turnMotor           the CANSparkMax object representing the turn motor
   *                            of the swerve module
   * @param absoluteEncoder     the CANcoder object representing the absolute
   *                            encoder of the swerve module
   * @param initialTurnPosition the initial position of the turn encoder
   * @param turnMultiplier      the multiplier for the turn encoder (for gears)
   *                            Reccomended 150/7:1
   * @param driveMotor          the CANSparkMax object representing the drive
   *                            motor of the swerve module
   * @param maxDriveSpeed       the maximum speed of the drive motor
   * @param amps                an array of integers representing the current
   *                            limits for the turn and drive motors
   * @param turnP               the proportional gain for the turn motor's PID
   *                            controller
   * @param turnI               the integral gain for the turn motor's PID
   *                            controller
   * @param turnD               the derivative gain for the turn motor's PID
   *                            controller
   */
  public SwerveModule(CANSparkMax turnMotor, CANcoder absoluteEncoder,
      double initialTurnPosition, double turnMultiplier,
      CANSparkMax driveMotor, double maxDriveSpeed, int[] amps,
      double turnP, double turnI, double turnD, double turnFF, int moduleNumber) {

    this.moduleNumber = moduleNumber;

    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;

    setDriveAmps(amps[1]);
    this.driveEncoder = driveMotor.getEncoder();

    this.turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    setTurnAmps(amps[0]);
    this.turnEncoder = turnMotor.getEncoder();
    this.absoluteEncoder = absoluteEncoder;
    this.turnEncoder.setPosition(
        absoluteEncoder.getPosition().getValue() * 360 - initialTurnPosition);
    this.turnEncoder.setPositionConversionFactor(turnMultiplier);

    turnPID = turnMotor.getPIDController();
    turnPID.setP(turnP);
    turnPID.setI(turnI);
    turnPID.setD(turnD);
    turnPID.setFF(turnFF);
    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(-180);
    turnPID.setPositionPIDWrappingMaxInput(180);

    burn();
  }

  /** Burn flash on all motor controllers */
  public void burn() {
    turnMotor.burnFlash();
    driveMotor.burnFlash();
  }

  /** Set position in degrees */
  public void setTurnPosition(double position) {
    targetTurnPosition = unwrap(position);
  }

  /** Returns in degrees */
  public double getTargetTurnPosition() {
    return targetTurnPosition;
  }

  public void setPower(double power) {
    targetPower = power;
  }

  /** Returns in degrees */
  public double getTurnPosition() {
    return turnEncoder.getPosition();
  }

  public RelativeEncoder getTurnEncoder() {
    return turnEncoder;
  }

  public RelativeEncoder getDriveEncoder() {
    return driveEncoder;
  }

  public void setTurnAmps(int amps) {
    turnMotor.setSmartCurrentLimit(amps);
  }

  public void setDriveAmps(int amps) {
    driveMotor.setSmartCurrentLimit(amps);
  }

  private double unwrap(double angle) {
    while (angle > 180) {
      angle -= 360;
    }
    while (angle < -180) {
      angle += 360;
    }
    return angle;
  }

  /** If value is greater than 180, it will be wrapped to -180 */
  private double unWrapError() {
    double error = targetTurnPosition - getTurnPosition();

    return unwrap(error);
  }

  public void setBrakeMode(boolean brake) {
    driveMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  /** CALL IN PERIODIC */
  public void update() {
    /*
     * Find closest path to target angle
     * If error is greater than 90 degrees,
     * then face opposite direction
     * Reversing drive motor is handled by the cosine function in the drivePID
     */
    double position = unWrapError();
    double absPos = Math.abs(position);
    double targetAngle = targetTurnPosition;
    if (absPos > 180 / 2) {
      targetAngle += 180;
    }
    turnPID.setReference(targetAngle, ControlType.kSmartMotion);

    double cosErr = (Math.cos(Units.degreesToRadians(position)));
    if (Math.abs(cosErr) < 0.1) {
      cosErr = 0;
    }

    driveMotor.set(cosErr * targetPower);

    SmartDashboard.putNumber("Module " + moduleNumber + " Angle", getTurnPosition());
    SmartDashboard.putNumber("Module " + moduleNumber + " Angle Error", targetAngle - getTurnPosition());
    SmartDashboard.putNumber("Module " + moduleNumber + " Power", targetPower);
  }
}