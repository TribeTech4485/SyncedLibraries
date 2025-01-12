// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SyncedLibraries.SystemBases.Estopable;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

/** Represents a swerve drive style drivetrain. */
public abstract class SwerveDriveBase extends Estopable {
  /** Wheel speed */
  public static final double kMaxSpeed = 1.0; // TODO: increase max speed
  public static final double kMaxAngularSpeed = (2 * Math.PI) / 4; // 1/4 rotation per second

  // The physical dimensions of the robot
  protected final double _robotWidth;
  protected final double _robotLength;
  protected final double _robotWidthOffset;
  protected final double _robotLengthOffset;
  protected final Translation2d m_frontLeftLocation;
  protected final Translation2d m_frontRightLocation;
  protected final Translation2d m_backLeftLocation;
  protected final Translation2d m_backRightLocation;
  protected final Translation2d m_frontLocation;
  protected final Translation2d m_rightLocation;
  protected final Translation2d m_backLocation;
  protected final Translation2d m_leftLocation;

  protected final SwerveModuleBase m_frontLeft;
  protected final SwerveModuleBase m_frontRight;
  protected final SwerveModuleBase m_backLeft;
  protected final SwerveModuleBase m_backRight;
  protected final SwerveModuleBase[] modules;

  protected final AHRS m_gyro;
  protected final SwerveDriveKinematics m_kinematics;
  protected final SwerveDriveOdometry m_odometry;
  protected StructArrayPublisher<SwerveModuleState> NetworkTablesSwervePublisherDesired;
  protected StructArrayPublisher<SwerveModuleState> NetworkTablesSwervePublisherCurrent;

  protected final SwerveModuleState[] lockPositions = generateLockPositions();
  protected SwerveModuleState[] swerveModuleStates = lockPositions;

  protected boolean locked = false;
  protected boolean fieldRelative = true;
  protected boolean brakeMode = false;
  protected boolean slowMode = false;
  protected boolean sudoMode = false;

  public final double[] modulesDrivePID;
  public final double[] modulesTurnPID;
  public final int driveAmps = 10;
  public final int turnAmps = 10;

  protected final PIDController turnController;

  int counter = 0;

  /**
   * An instance for controlling a swerve drivetrain
   * 
   * @param width   The width of the robot in meters
   * @param length  The length of the robot in meters
   * @param modules An array of SwerveModules in the order of front left, front
   *                right, back left, back right
   * @param swerveDrivePID The PID values for the drive motors
   * @param swerveTurnPID  The PID values for the turn motors
   * @param botTurnPID     The PID values for the bot turning
   * @param driveAmps      The max amps for the drive motors
   * @param turnAmps       The max amps for the turn motors
   */
  public SwerveDriveBase(double width, double length, SwerveModuleBase[] modules,
      double[] swerveDrivePID, double[] swerveTurnPID, double[] botTurnPID, int driveAmps, int turnAmps) {
    NetworkTablesSwervePublisherDesired = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/DesiredSwerveStates", SwerveModuleState.struct).publish();
    NetworkTablesSwervePublisherCurrent = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/CurrentSwerveStates", SwerveModuleState.struct).publish();

    _robotWidth = width;
    _robotLength = length;
    _robotWidthOffset = _robotWidth / 2;
    _robotLengthOffset = _robotLength / 2;

    m_frontLeftLocation = new Translation2d(_robotLengthOffset, _robotWidthOffset);
    m_frontRightLocation = new Translation2d(_robotLengthOffset, -_robotWidthOffset);
    m_backLeftLocation = new Translation2d(-_robotLengthOffset, _robotWidthOffset);
    m_backRightLocation = new Translation2d(-_robotLengthOffset, -_robotWidthOffset);
    m_frontLocation = new Translation2d(_robotLengthOffset, 0);
    m_rightLocation = new Translation2d(0, -_robotWidthOffset);
    m_backLocation = new Translation2d(-_robotLengthOffset, 0);
    m_leftLocation = new Translation2d(0, _robotWidthOffset);

    m_frontLeft = modules[0];
    m_frontRight = modules[1];
    m_backLeft = modules[2];
    m_backRight = modules[3];
    this.modules = modules;

    m_gyro = new AHRS();
    m_gyro.reset();

    m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

        
        modulesDrivePID = swerveDrivePID;
        modulesTurnPID = swerveTurnPID;
        turnController = new PIDController(botTurnPID[0], botTurnPID[1], botTurnPID[2]);
        turnController.enableContinuousInput(0, Math.PI * 2);
  }

  public void enableXLock() {
    locked = true;
    setDriveBrakeMode(true);
    swerveModuleStates = lockPositions;
    setDesiredStates();
  }

  public void disableXLock() {
    locked = false;
    setDriveBrakeMode(brakeMode);
  }

  protected void setDriveBrakeMode(boolean brakeMode) {
    this.brakeMode = brakeMode; // Save the brake mode for when we disable X Lock
    m_frontLeft.setDriveBrakeMode(brakeMode);
    m_frontRight.setDriveBrakeMode(brakeMode);
    m_backLeft.setDriveBrakeMode(brakeMode);
    m_backRight.setDriveBrakeMode(brakeMode);
  }

  /**
   * Method to drive the robot using joystick info.<br>
   * All speeds are from -1 to 1.<br>
   * Uses power level for rotation
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (right).
   * @param rotationSpeed Angular rate of the robot.
   */
  public void inputDrivingX_Y(double xSpeed, double ySpeed,
      double rotationSpeed) {
    inputDrivingX_Y(xSpeed, ySpeed, rotationSpeed, -1);
  }

  /**
   * Method to drive the robot using joystick info.<br>
   * All speeds are from -1 to 1.<br>
   * Uses power level for rotation
   *
   * @param xSpeed              Speed of the robot in the x direction (forward).
   * @param ySpeed              Speed of the robot in the y direction (right).
   * @param rotationSpeed       Angular rate of the robot.
   * @param centerOfRotationPOV Input pov value where -1 is center, and 0 is front, clockwise degrees
   */
  public void inputDrivingX_Y(double xSpeed, double ySpeed,
      double rotationSpeed, int centerOfRotationPOV) {

    if (!locked) {
      xSpeed *= kMaxSpeed;
      ySpeed *= kMaxSpeed;
      rotationSpeed *= kMaxAngularSpeed;
      Translation2d centerOfRotation = POVToTranslate2d(centerOfRotationPOV);

      // Calculate the swerve module states from the requested speeds
      swerveModuleStates = m_kinematics.toSwerveModuleStates(
          // Account for time between updates
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xSpeed, -ySpeed, rotationSpeed,
                  fieldRelative ? m_gyro.getRotation2d() : Rotation2d.fromDegrees(0)),
              0.02),
          centerOfRotation);
    }
  }

  /**
   * Method to drive the robot using joystick info.<br>
   * All speeds are from -1 to 1.<br>
   * Uses desired angle for rotation
   *
   * @param xSpeed      Speed of the robot in the x direction (forward).
   * @param ySpeed      Speed of the robot in the y direction (right).
   * @param desiredTheta Desired angle of the robot in radians.
   * @param centerOfRotationPOV Input pov value where -1 is center, and 0 is front, clockwise degrees
   */
  public void inputDrivingX_Y_A(double xSpeed, double ySpeed, double desiredTheta, int centerOfRotationPOV) {
    turnController.setSetpoint(desiredTheta);
    inputDrivingX_Y(xSpeed, ySpeed, turnController.calculate(Math.toRadians(m_gyro.getYaw() % 360)));
  }

  /** Used for using a POV joystick to rotate around corner of robot */
  protected Translation2d POVToTranslate2d(int centerOfRotation) {
    Translation2d rotationCenter;
    switch (centerOfRotation) {
      case -1:
        rotationCenter = new Translation2d(0, 0);
        break;
      case 0:
        rotationCenter = m_frontLocation;
        break;
      case 45:
        rotationCenter = m_frontRightLocation;
        break;
      case 90:
        rotationCenter = m_rightLocation;
        break;
      case 135:
        rotationCenter = m_backRightLocation;
        break;
      case 180:
        rotationCenter = m_backLocation;
        break;
      case 225:
        rotationCenter = m_backLeftLocation;
        break;
      case 270:
        rotationCenter = m_leftLocation;
        break;
      case 315:
        rotationCenter = m_frontLeftLocation;
        break;
      default:
        rotationCenter = new Translation2d(0, 0);
        break;
    }
    return rotationCenter;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  @Override
  public void periodic() {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    setDesiredStates();

    NetworkTablesSwervePublisherDesired.set(swerveModuleStates);
    NetworkTablesSwervePublisherCurrent.set(
        new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        });

    SmartDashboard.putData("Gyro", m_gyro);
    updateOdometry();
  }

  protected void setDesiredStates() {
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    for (SwerveModuleBase module : modules) {
      if (module.getSudoMode() != sudoMode) {
        module.setSudoMode(sudoMode);
      }
      if (module.getSlowMode() != slowMode) {
        module.setSlowMode(slowMode);
      }
    }
  }

  public void setFieldRelative(boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
  }

  public boolean getFieldRelative() {
    return fieldRelative;
  }

  public void setSlowMode(boolean slowMode) {
    this.slowMode = slowMode;
  }

  public void setSudoMode(boolean sudoMode) {
    this.sudoMode = sudoMode;
  }

  public void resetGyro() {
    m_gyro.zeroYaw();
  }

  public void setDriveAmps(int amps) {
    for (SwerveModuleBase module : modules) {
      module.setDriveAmps(amps);
    }
  }

  public void setTurnAmps(int amps) {
    for (SwerveModuleBase module : modules) {
      module.setTurnAmps(amps);
    }
  }

  /**
   * Positions to turn all the wheels inward
   * <p>
   * aka X Lock
   */
  private SwerveModuleState[] generateLockPositions() {
    SwerveModuleState[] lockPositions = new SwerveModuleState[4];
    lockPositions[0] = new SwerveModuleState(0, new Rotation2d(1 * Math.PI / 4));
    lockPositions[1] = new SwerveModuleState(0, new Rotation2d(7 * Math.PI / 4));
    lockPositions[2] = new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4));
    lockPositions[3] = new SwerveModuleState(0, new Rotation2d(5 * Math.PI / 4));
    return lockPositions;
  }

  public void stop() {
    inputDrivingX_Y(0, 0, 0, -1);
  }

  /**
   * Stops all motors and sets the robot to a safe state.<br>
   * <b>ONLY FOR USE IN EMERGENCY</b>
   * <p>
   * Hopefully won't skid too far
   */
  public void ESTOP() {
    enableXLock();
  }
}