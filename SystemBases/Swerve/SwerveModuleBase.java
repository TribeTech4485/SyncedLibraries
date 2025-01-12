// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases.Swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModuleBase extends SubsystemBase {
  protected SwerveDriveBase driveTrainBase;
  protected final CANSparkMax m_driveMotor;
  protected final CANSparkMax m_turningMotor;

  protected final RelativeEncoder m_driveEncoder;
  protected final SparkAbsoluteEncoder m_turningEncoder;

  protected final PIDController m_drivePIDController;
  protected final PIDController m_turnPIDController;

  protected final double driveGearRatio = 1 / (10 * Math.PI * 15 / 50); // 1 is the gear ratio when I find out

  protected boolean sudoMode = false;
  protected boolean slowMode = false;

  /**
   * Constructs a new SwerveModule.
   * MUST CALL {@link #inputDriveTrain(SwerveDriveBase)} AFTER CONSTRUCTION
   *
   * @param driveMotor    The motor that drives the module.
   * @param turningMotor  The motor that turns the module.
   * @param turningOffset The offset for the turning encoder. Starting position
   * @param name          The name of the module. Ie. "Front Left"
   */
  public SwerveModuleBase(CANSparkMax driveMotor, CANSparkMax turningMotor, double turningOffset, String name) {
    this.setName(name);

    // DRIVE MOTOR SETUP
    m_driveMotor = driveMotor;

    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(driveGearRatio);
    m_driveEncoder.setVelocityConversionFactor(driveGearRatio);

    m_drivePIDController = new PIDController(0, 0, 0);

    // TURNING MOTOR SETUP
    m_turningMotor = turningMotor;

    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();
    m_turningEncoder.setZeroOffset(turningOffset);

    m_turnPIDController = new PIDController(0, 0, 0);
    m_turnPIDController.enableContinuousInput(0, 1);

    m_turningMotor.burnFlash();
    m_driveMotor.burnFlash();
  }

  public void inputDriveTrain(SwerveDriveBase driveTrainBase) {
    this.driveTrainBase = driveTrainBase;
    // drive motor setup
    m_driveMotor.setSmartCurrentLimit(driveTrainBase.driveAmps);
    double[] drivePIDS = driveTrainBase.modulesDrivePID;
    m_drivePIDController.setPID(drivePIDS[0], drivePIDS[1], drivePIDS[2]);

    // turning motor setup
    m_turningMotor.setSmartCurrentLimit(driveTrainBase.turnAmps);
    double[] turnPIDS = driveTrainBase.modulesTurnPID;
    m_turnPIDController.setPID(turnPIDS[0], turnPIDS[1], turnPIDS[2]);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // return new SwerveModuleState(
    // m_driveEncoder.getVelocity(), new
    // Rotation2d(m_turningEncoder.getPosition()));
    return new SwerveModuleState(
        m_driveMotor.get(), getEncoderPos());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), getEncoderPos());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d encoderRotation = getEncoderPos();

    SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    // Optimize the reference state to avoid spinning further than 90 degrees
    state = SwerveModuleState.optimize(state, getEncoderPos());

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired direction of travel that can occur when modules change
    // directions. This results in smoother driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // set the wanted position, actual moving done in periodic
    m_drivePIDController.setSetpoint(state.speedMetersPerSecond);
    m_turnPIDController.setSetpoint(covertFromRadians(state.angle.getRadians()));
  }

  @Override
  public void periodic() {
    double drivingPower = 0;
    double turnPower = m_turnPIDController.calculate(m_turningEncoder.getPosition());

    if (sudoMode) {
      if (Math.abs(m_drivePIDController.getSetpoint()) > 0.05) {
        drivingPower = Math.signum(m_drivePIDController.getSetpoint());
      } else {
        drivingPower = 0;
      }
    } else {
      if (slowMode) {
        drivingPower = m_drivePIDController.calculate(m_driveEncoder.getVelocity() * 0.5);
      } else {
        drivingPower = m_drivePIDController.calculate(m_driveEncoder.getVelocity());
      }
    }

    m_turningMotor.set(turnPower);
    m_driveMotor.set(drivingPower);

    SmartDashboard.putData(this.getName() + " swerve turning PID", m_turnPIDController);
    SmartDashboard.putNumber(this.getName() + " swerve turning encoder", m_turningEncoder.getPosition());
    SmartDashboard.putNumber(this.getName() + " swerve turning degrees", getEncoderPos().getDegrees());
    SmartDashboard.putNumber(this.getName() + " swerve turning power", m_turningMotor.get());

    SmartDashboard.putData(this.getName() + " swerve driving PID", m_drivePIDController);
    SmartDashboard.putNumber(this.getName() + " swerve driving speed", m_driveEncoder.getVelocity());
    SmartDashboard.putNumber(this.getName() + " swerve driving power", m_driveMotor.get());
  }

  public void setDriveBrakeMode(boolean brake) {
    if (brake) {
      m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    } else {
      m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
  }

  protected double covertFromRadians(double radians) {
    return (radians / (-2 * Math.PI)) + (1 / 2);
  }

  protected double convertToRadians(double position) {
    return (-(position - (1 / 2))) * (2 * Math.PI);
  }

  /** Converts to radians */
  public Rotation2d getEncoderPos() {
    return new Rotation2d(convertToRadians(m_turningEncoder.getPosition()));
  }

  public CANSparkMax getTurnMotor() {
    return m_turningMotor;
  }

  public CANSparkMax getDriveMotor() {
    return m_driveMotor;
  }

  public RelativeEncoder getDriveEncoder() {
    return m_driveEncoder;
  }

  public SparkAbsoluteEncoder getTurnEncoder() {
    return m_turningEncoder;
  }

  public PIDController getDrivePIDController() {
    return m_drivePIDController;
  }

  public PIDController getTurnPIDController() {
    return m_turnPIDController;
  }

  public boolean getSudoMode() {
    return sudoMode;
  }

  public void setSudoMode(boolean sudoMode) {
    this.sudoMode = sudoMode;
    if (sudoMode) {
      
    }
  }

  public boolean getSlowMode() {
    return slowMode;
  }

  public void setSlowMode(boolean slowMode) {
    this.slowMode = slowMode;
  }

  public void setDriveAmps(int limit) {
    m_driveMotor.setSmartCurrentLimit(limit);
  }

  public void setTurnAmps(int limit) {
    m_turningMotor.setSmartCurrentLimit(limit);
  }
}