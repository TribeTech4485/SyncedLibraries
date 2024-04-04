// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases;

import com.revrobotics.CANSparkBase.ControlType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** For info about positions and standards, read comment inside */
public class SwerveDriveBase extends SubsystemBase {
  /*
   * Motor Positions:
   * Front
   * 0 ---- 1
   * |......|
   * |......|
   * 3 ---- 2
   * Rear
   * 
   * For directions, 0 is forward,
   * positive is clockwise, negative is counter-clockwise
   * 180 is reverse
   */

  private SwerveModule[] modules;

  /**
   * Represents a swerve drive base that consists of multiple swerve modules.
   * <p>
   * All arrays must be of length 4. For more information see comment inside.
   * 
   * @param turnMotors         The motors that control the turning of the
   *                           wheels.
   * @param turnEncoders       The encoders that measure the turning of the
   *                           wheels.
   * @param absoluteEncoders   The encoders that measure the absolute turning of
   *                           the wheels.
   * @param absoluteTurnOffset The initial positions of the turn motors in
   *                           degrees.
   * @param driveMotors        The motors that control the driving of the
   *                           wheels.
   * @param maxDriveSpeed      The maximum speed of the drive motors.
   * @param amps               The current limits for the motors in the format
   *                           of [turn, drive].
   * @param turnP              The proportional gain for the turn PID.
   * @param turnI              The integral gain for the turn PID.
   * @param turnD              The derivative gain for the turn PID.
   */
  public SwerveDriveBase(CANSparkMax[] turnMotors, CANcoder[] absoluteEncoders,
      double[] absoluteTurnOffset, double turnMultiplier,
      CANSparkMax[] driveMotors, double maxDriveSpeed, int[] amps,
      double turnP, double turnI, double turnD) {

    modules = new SwerveModule[4];

    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(turnMotors[i], absoluteEncoders[i],
          absoluteTurnOffset[i], turnMultiplier,
          driveMotors[i], maxDriveSpeed, amps,
          turnP, turnI, turnD);
    }
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.update();
    }
  }

  /** @param direction angle in degrees from positive x direction */
  public void update(double power, double direction, double rotation) {
    double[] directions = new double[4];
    double[] speeds = new double[4];
    for (int i = 0; i < 4; i++) {
      // TODO: Find what directions to turn to
      directions[i] = 0;

      // TODO: Find what speeds to drive at
      speeds[i] = 0.5;

      modules[i].setTurnPosition(directions[i]);
      modules[i].setPower(speeds[i]);
    }
  }

  public SwerveModule[] getModules() {
    return modules;
  }

  public SwerveModule getModule(int index) {
    return modules[index];
  }

  public void setDriveAmps(int amps) {
    for (SwerveModule module : modules) {
      module.setDriveAmps(amps);
    }
  }

  public void setTurnAmps(int amps) {
    for (SwerveModule module : modules) {
      module.setTurnAmps(amps);
    }
  }

  public void x_lock() {
    for (int i = 0; i < 4; i++) {
      modules[i].setTurnPosition((315 + i * 270) % 360);
    }
    brakeMode(true);
  }

  public void brakeMode(boolean brake) {
    for (SwerveModule module : modules) {
      module.setBrakeMode(brake);
    }
  }

  private class SwerveModule {
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder turnEncoder;
    private RelativeEncoder driveEncoder;
    private SparkPIDController turnPID;
    private double targetTurnPosition;
    private double targetPower;

    /**
     * Constructs a SwerveModule object with the specified parameters.
     *
     * @param turnMotor           the CANSparkMax object representing the turn motor
     *                            of the swerve module
     * @param absoluteEncoder     the CANcoder object representing the absolute
     *                            encoder of the swerve module
     * @param initialTurnPosition the initial position of the turn encoder
     * @param turnMultiplier      the multiplier for the turn encoder (for gears)
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
        double turnP, double turnI, double turnD) {

      this.driveMotor = driveMotor;
      this.turnMotor = turnMotor;

      setDriveAmps(amps[1]);
      this.driveEncoder = driveMotor.getEncoder();

      setTurnAmps(amps[0]);
      this.turnEncoder = turnMotor.getEncoder();
      this.turnEncoder.setPosition(
          absoluteEncoder.getPosition().getValue() * 360 - initialTurnPosition);
      this.turnEncoder.setPositionConversionFactor(turnMultiplier);

      turnPID = turnMotor.getPIDController();
      turnPID.setP(turnP);
      turnPID.setI(turnI);
      turnPID.setD(turnD);
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
      targetTurnPosition = position;
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

    /** If value is greater than 180, it will be wrapped to -180 */
    private double unWrapError() {
      double error = getTargetTurnPosition() - getTurnPosition();
      while (error > 180) {
        error -= 360;
      }
      while (error < -180) {
        error += 360;
      }
      return error;
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
      double error = Math.abs(position);
      if (error > 180 / 2) {
        targetTurnPosition += 180;
      }
      turnPID.setReference(targetTurnPosition, ControlType.kSmartMotion);

      double cosErr = (Math.cos(Units.degreesToRadians(position)));
      if (Math.abs(cosErr) < 0.1) {
        cosErr = 0;
      }

      driveMotor.set(cosErr * targetPower);
    }
  }
}
