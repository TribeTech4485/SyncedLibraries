// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SyncedLibraries.DrivetrainMode;

/** Represents a swerve drive style drivetrain. */
public class SwerveDriveBase extends SubsystemBase {
  public static final double kMaxSpeed = 1.0; // 3 meters per second // TODO: increase
  public static final double kMaxAngularSpeed = 2 * Math.PI / 4; // 1/2 rotation per second

  // For a square 3ft x 3ft robot, the wheelbase is 0.381 meters from center to
  // module.
  private final double _robotWidth;
  private final double _robotLength;
  private final double _robotWidthOffset;
  private final double _robotLengthOffset;

  private final Translation2d m_frontLeftLocation;
  private final Translation2d m_frontRightLocation;
  private final Translation2d m_backLeftLocation;
  private final Translation2d m_backRightLocation;
  private final Translation2d m_frontLocation;
  private final Translation2d m_rightLocation;
  private final Translation2d m_backLocation;
  private final Translation2d m_leftLocation;

  private final SwerveModuleBase m_frontLeft;
  private final SwerveModuleBase m_frontRight;
  private final SwerveModuleBase m_backLeft;
  private final SwerveModuleBase m_backRight;

  int counter = 0;

  private final AHRS m_gyro;
  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;
  private StructArrayPublisher<SwerveModuleState> NetworkTablesSwervePublisherDesired;
  private StructArrayPublisher<SwerveModuleState> NetworkTablesSwervePublisherCurrent;

  private final SwerveModuleState[] lockPositions = generateLockPositions();
  private SwerveModuleState[] swerveModuleStates = lockPositions;

  private DrivetrainMode drivetrainMode = DrivetrainMode.NORMAL;
  private boolean fieldRelative = true;
  private boolean brakeMode = false;

  public boolean allowTurnMotors = true;
  public boolean allowDriveMotors = true;

  /**
   * An instance for controlling a swerve drivetrain
   * 
   * @param width   The width of the robot in feet
   * @param length  The length of the robot in feet
   * @param modules An array of SwerveModules in the order of front left, front
   *                right, back left, back right
   */
  public SwerveDriveBase(double width, double length, SwerveModuleBase[] modules) {
    NetworkTablesSwervePublisherDesired = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/DesiredSwerveStates", SwerveModuleState.struct).publish();
    NetworkTablesSwervePublisherCurrent = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/CurrentSwerveStates", SwerveModuleState.struct).publish();

    _robotWidth = Units.feetToMeters(width);
    _robotLength = Units.feetToMeters(length);
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
  }

  public void enableXLock() {
    drivetrainMode = DrivetrainMode.LOCKED;
    setDriveBrakeMode(true);
    swerveModuleStates = lockPositions;
    setDesiredStates();
  }

  public void disableXLock() {
    drivetrainMode = DrivetrainMode.NORMAL;
    setDriveBrakeMode(brakeMode);
  }

  private void setDriveBrakeMode(boolean brakeMode) {
    this.brakeMode = brakeMode; // Save the brake mode for when we disable X Lock
    m_frontLeft.setDriveBrakeMode(brakeMode);
    m_frontRight.setDriveBrakeMode(brakeMode);
    m_backLeft.setDriveBrakeMode(brakeMode);
    m_backRight.setDriveBrakeMode(brakeMode);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed              Speed of the robot in the x direction (forward).
   * @param ySpeed              Speed of the robot in the y direction (right).
   * @param rotationSpeed       Angular rate of the robot.
   * @param fieldRelative       Whether the provided x and y speeds are relative
   *                            to the field.
   * @param centerOfRotationPOV Input pov value where -1 is center, and 0 is front
   */
  public void inputDrivingX_Y(double xSpeed, double ySpeed,
      double rotationSpeed, int centerOfRotationPOV) {

    if (drivetrainMode == DrivetrainMode.NORMAL) {
      xSpeed *= kMaxSpeed;
      ySpeed *= kMaxSpeed;
      rotationSpeed *= kMaxAngularSpeed;
      Translation2d centerOfRotation = POVToTranslate2d(centerOfRotationPOV);

      swerveModuleStates = m_kinematics.toSwerveModuleStates(
          // Account for time between updates
          ChassisSpeeds.discretize(
              fieldRelative
                  // if field relative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed, -ySpeed, rotationSpeed,
                      m_gyro.getRotation2d())
                  // if robot relative
                  : new ChassisSpeeds(xSpeed, -ySpeed, rotationSpeed),
              0.02),
          centerOfRotation);
    }
  }

  private void setDesiredStates() {
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Used for using a POV joystick to rotate around corner of robot */
  private Translation2d POVToTranslate2d(int centerOfRotation) {
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
    if (allowTurnMotors) {
      setDesiredStates();
    }

    NetworkTablesSwervePublisherDesired.set(swerveModuleStates);
    NetworkTablesSwervePublisherCurrent.set(
        new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        });

    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro Rad", m_gyro.getAngle() / 180 * Math.PI);
    updateOdometry();
  }

  private SwerveModuleState[] generateLockPositions() {
    SwerveModuleState[] lockPositions = new SwerveModuleState[4];
    lockPositions[0] = new SwerveModuleState(0, new Rotation2d(1 * Math.PI / 4));
    lockPositions[1] = new SwerveModuleState(0, new Rotation2d(7 * Math.PI / 4));
    lockPositions[2] = new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4));
    lockPositions[3] = new SwerveModuleState(0, new Rotation2d(5 * Math.PI / 4));
    return lockPositions;
  }
}