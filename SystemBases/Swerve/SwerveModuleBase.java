// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases.Swerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SyncedLibraries.SystemBases.Estopable;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;
import frc.robot.SyncedLibraries.SystemBases.Utils.VelocityFFController;

public abstract class SwerveModuleBase extends Estopable {
  protected final SparkMax m_driveMotor;
  protected final SparkMax m_turningMotor;

  protected final RelativeEncoder m_driveEncoder;
  protected final SparkAbsoluteEncoder m_turningEncoder;

  // protected final TrapezoidProfile.Constraints m_driveConstraints;
  // protected final ProfiledPIDController m_drivePIDController;
  protected final PIDController m_turnPIDController;
  // protected final SimpleMotorFeedforward m_driveFeedforward;
  protected final VelocityFFController driveVelFFController;
  protected final PIDConfig turnPIDF;

  // protected final double driveGearRatio = 1 / (10 * Math.PI * 15 / 50); // 1 is
  // the gear ratio when I find out

  protected boolean sudoMode = false;
  protected boolean slowMode = false;
  protected boolean voltageControlMode = false;

  protected double slowModeMultiplier = 0.5;

  protected int driveAmps;
  protected int turnAmps;
  protected int breakerMaxAmps = 30;

  /**
   * @param driveMotor       The motor that drives the module.
   * @param turningMotor     The motor that turns the module.
   * @param turningOffset    The offset for the turning encoder. Starting position
   * @param name             The name of the module. Ie. "Front Left"
   * @param driveConfig      The configuration for the drive motor.
   * @param turningConfig    The configuration for the turning motor.
   * @param drivePIDF        The PIDF values for the drive motor: P, I, D, S, V, A
   * @param turnPID          The PID values for the turning motor: P, I, D
   * @param driveConstraints The constraints for the drive motor.
   */
  public SwerveModuleBase(SparkMax driveMotor, SparkMax turningMotor, double turningOffset, String name,
      SparkBaseConfig driveConfig, SparkBaseConfig turningConfig,
      PIDConfig drivePIDF, PIDConfig turnPID) {
    this.setName(name);

    // DRIVE MOTOR SETUP
    m_driveMotor = driveMotor;

    m_driveMotor.configure(driveConfig
    // .positionConversionFactor(driveGearRatio)
    // .velocityConversionFactor(driveGearRatio))
        ,
        ResetMode.kResetSafeParameters, // resetToFactoryDefaults()
        PersistMode.kPersistParameters); // burnFlash()

    m_driveEncoder = m_driveMotor.getEncoder();
    driveVelFFController = new VelocityFFController(drivePIDF, RotationsPerSecond);

    // TURNING MOTOR SETUP
    m_turningMotor = turningMotor;

    m_turningMotor.configure(turningConfig.apply(
        new AbsoluteEncoderConfig().zeroOffset(
            turningOffset)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

    this.turnPIDF = turnPID;
    m_turnPIDController = new PIDController(0, 0, 0);
    m_turnPIDController.enableContinuousInput(0, 2 * Math.PI);

    driveAmps = m_driveMotor.configAccessor.getSmartCurrentLimit();
    turnAmps = m_turningMotor.configAccessor.getSmartCurrentLimit();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), getEncoderPos());
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
    // SwerveModuleState state = desiredState;

    state.optimize(encoderRotation);
    state.cosineScale(encoderRotation);

    // set the wanted position, actual moving done in periodic
    if (slowMode && !voltageControlMode) {
      driveVelFFController.setGoal(RotationsPerSecond.of(state.speedMetersPerSecond * slowModeMultiplier));
    } else {
      driveVelFFController.setGoal(RotationsPerSecond.of(state.speedMetersPerSecond));
    }
    if (state.speedMetersPerSecond == 0 && m_driveEncoder.getVelocity() >= 0.1 && !voltageControlMode) {
      // slowing down but not stopped yet (and not in voltage control mode)
    } else {
      m_turnPIDController.setSetpoint(state.angle.getRadians());
    }

    // m_driveMotor.getClosedLoopController().setReference(state.speedMetersPerSecond,
    // ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void periodic() {
    turnPIDF.applyTo(m_turnPIDController);

    double drivingVoltage = 0;
    double turningVoltage = m_turnPIDController.calculate(m_turningEncoder.getPosition());

    if (true) {
      if (voltageControlMode) {
        if (sudoMode) {
          // Sudo mode, drive motors will be at 12 volts
          if (Math.abs(driveVelFFController.getGoal().in(RotationsPerSecond)) > 0.05) {
            drivingVoltage = Math.signum(driveVelFFController.getGoal().in(RotationsPerSecond)) * 12;
          } else {
            drivingVoltage = 0;
          }
        } else {
          // Using voltage control, but not sudo mode
          drivingVoltage = driveVelFFController.getGoal().in(RotationsPerSecond);
        }
      } else {
        drivingVoltage = driveVelFFController.calculate(RotationsPerSecond.of(m_driveEncoder.getVelocity()))
            .magnitude();
      }
    }

    m_turningMotor.setVoltage(-turningVoltage);
    m_driveMotor.setVoltage(drivingVoltage);
    // m_driveMotor.setVoltage(0);

    SmartDashboard.putData(getName() + " swerve turning PID", m_turnPIDController);
    SmartDashboard.putNumber(getName() + " swerve turning encoder", m_turningEncoder.getPosition());
    SmartDashboard.putNumber(getName() + " swerve turning degrees", getEncoderPos().getDegrees());
    SmartDashboard.putNumber(getName() + " swerve turning power", m_turningMotor.get());

    SmartDashboard.putData(getName() + " swerve driving PID", driveVelFFController);
    SmartDashboard.putNumber(getName() + " swerve driving speed", m_driveEncoder.getVelocity());
    SmartDashboard.putNumber(getName() + " swerve driving power", m_driveMotor.get());
  }

  public void setDriveBrakeMode(boolean brake) {
    m_driveMotor.configure(new SparkMaxConfig().idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Converts to radians */
  public Rotation2d getEncoderPos() {
    return new Rotation2d(m_turningEncoder.getPosition());
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

  public VelocityFFController getDrivePIDController() {
    return driveVelFFController;
  }

  public PIDController getTurnPIDController() {
    return m_turnPIDController;
  }

  public boolean getSudoMode() {
    return sudoMode;
  }

  public void setSudoMode(boolean sudoMode) {
    this.sudoMode = sudoMode;
    m_driveMotor.configure(new SparkMaxConfig()
        .smartCurrentLimit(sudoMode ? breakerMaxAmps : driveAmps),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_turningMotor.configure(new SparkMaxConfig()
        .smartCurrentLimit(sudoMode ? breakerMaxAmps : turnAmps),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    setVoltageControlMode(sudoMode);
  }

  public boolean getSlowMode() {
    return slowMode;
  }

  public void setSlowMode(boolean slowMode) {
    this.slowMode = slowMode;
  }

  public void setSlowModeMultiplier(double multiplier) {
    slowModeMultiplier = multiplier;
  }

  /** Danger, drive motors will interprit the speeds as voltages!! */
  public void setVoltageControlMode(boolean voltageControl) {
    voltageControlMode = voltageControl;

  }

  public boolean getVoltageControlMode() {
    return voltageControlMode;
  }

  public void setDriveAmps(int limit) {
    m_driveMotor.configure(new SparkMaxConfig().smartCurrentLimit(limit),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    driveAmps = limit;
  }

  public void setTurnAmps(int limit) {
    m_turningMotor.configure(new SparkMaxConfig().smartCurrentLimit(limit),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    turnAmps = limit;
  }

  @Override
  public void ESTOP() {
    // handled in SwerveDriveBase
    driveVelFFController.pidController.reset(0);
  }

  @Override
  public void onDisable() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);

    m_driveMotor.configure(new SparkMaxConfig(),
        ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(new SparkMaxConfig(),
        ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public boolean getDriveBrakeMode() {
    return m_driveMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
  }
}