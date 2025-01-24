// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases.Swerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SyncedLibraries.SystemBases.Estopable;

public abstract class SwerveModuleBase extends Estopable {
  protected SwerveDriveBase driveTrainBase;
  protected final SparkMax m_driveMotor;
  protected final SparkMax m_turningMotor;

  protected final RelativeEncoder m_driveEncoder;
  protected final SparkAbsoluteEncoder m_turningEncoder;

  protected final PIDController m_drivePIDController;
  protected final PIDController m_turnPIDController;

  protected final double driveGearRatio = 1 / (10 * Math.PI * 15 / 50); // 1 is the gear ratio when I find out

  protected boolean sudoMode = false;
  protected boolean slowMode = false;

  protected int driveAmps;
  protected int turnAmps;

  /**
   * Constructs a new SwerveModule.
   * MUST CALL {@link #inputDriveTrain(SwerveDriveBase)} AFTER CONSTRUCTION
   *
   * @param driveMotor    The motor that drives the module.
   * @param turningMotor  The motor that turns the module.
   * @param turningOffset The offset for the turning encoder. Starting position
   * @param name          The name of the module. Ie. "Front Left"
   */
  public SwerveModuleBase(SparkMax driveMotor, SparkMax turningMotor, double turningOffset, String name) {
    this.setName(name);

    // DRIVE MOTOR SETUP
    m_driveMotor = driveMotor;

    m_driveMotor.configure(new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .apply(new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .apply(new EncoderConfig()
                .positionConversionFactor(driveGearRatio)
                .velocityConversionFactor(driveGearRatio))),
        ResetMode.kResetSafeParameters, // resetToFactoryDefaults()
        PersistMode.kPersistParameters); // burnFlash()

    m_driveEncoder = m_driveMotor.getEncoder();

    m_drivePIDController = new PIDController(0, 0, 0);

    // TURNING MOTOR SETUP
    m_turningMotor = turningMotor;

    m_turningMotor.configure(new SparkMaxConfig()
        .idleMode(IdleMode.kCoast)
        .apply(new AbsoluteEncoderConfig().zeroOffset(turningOffset)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

    m_turnPIDController = new PIDController(0, 0, 0);
    m_turnPIDController.enableContinuousInput(0, 1);
  }

  public void inputDriveTrain(SwerveDriveBase driveTrainBase) {
    this.driveTrainBase = driveTrainBase;
    // drive motor setup
    m_driveMotor.configure(new SparkMaxConfig().smartCurrentLimit(driveTrainBase.driveAmps),
        ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    double[] drivePIDS = driveTrainBase.modulesDrivePID;
    m_drivePIDController.setPID(drivePIDS[0], drivePIDS[1], drivePIDS[2]);

    // turning motor setup
    m_turningMotor.configure(new SparkMaxConfig().smartCurrentLimit(driveTrainBase.turnAmps),
        ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
    state.optimize(getEncoderPos());

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
    m_driveMotor.configure(new SparkMaxConfig().idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

  public SparkMax getTurnMotor() {
    return m_turningMotor;
  }

  public SparkMax getDriveMotor() {
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
      m_driveMotor.configure(new SparkMaxConfig().smartCurrentLimit(20), ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      m_turningMotor.configure(new SparkMaxConfig().smartCurrentLimit(20), ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    } else {
      m_driveMotor.configure(new SparkMaxConfig().smartCurrentLimit(driveAmps), ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      m_turningMotor.configure(new SparkMaxConfig().smartCurrentLimit(turnAmps), ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
  }

  public boolean getSlowMode() {
    return slowMode;
  }

  public void setSlowMode(boolean slowMode) {
    this.slowMode = slowMode;
  }

  public void setDriveAmps(int limit) {
    m_driveMotor.configure(new SparkMaxConfig().smartCurrentLimit(limit), ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    driveAmps = limit;
  }

  public void setTurnAmps(int limit) {
    m_turningMotor.configure(new SparkMaxConfig().smartCurrentLimit(limit), ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    turnAmps = limit;
  }

  @Override
  public void ESTOP() {
    // Handled by the SwerveDriveBase
  }

  @Override
  public void onDisable() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);

    m_driveMotor.configure(new SparkMaxConfig(), ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningMotor.configure(new SparkMaxConfig(), ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }
}