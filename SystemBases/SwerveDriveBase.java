// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private AHRS navx;
  private double speedTurnAngle = 45;

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
   * @param turnMultiplier     The multiplier for the turn encoders (for gears).
   *                           Recommended 150/7:1.
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
      double turnP, double turnI, double turnD, double turnFF) {

    modules = new SwerveModule[4];

    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(turnMotors[i], absoluteEncoders[i],
          absoluteTurnOffset[i], turnMultiplier,
          driveMotors[i], maxDriveSpeed, amps,
          turnP, turnI, turnD, turnFF, i);
    }
  }

  public void enableFieldCentric(AHRS navx) {
    this.navx = navx;
  }

  public void disableFieldCentric() {
    enableFieldCentric(null);
  }

  /** Angle at which the wheels will be if going full speed and full turn */
  public void setSpeedTurnAngle(double angle) {
    speedTurnAngle = angle;
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.update();
    }
  }

  /**
   * hello
   * Updates the swerve drive base with the specified parameters.
   * If inPolarCoords is true, a is the magnitude and b is the angle in degrees.
   * If inPolarCoords is false, a is the x component and b is the y component.
   * 
   * @param inPolarCoords Whether the inputs are in polar coordinates.
   * @param a             X or magnitude
   * @param b             Y or angle
   * @param rotation      The rotation
   */
  public void update(boolean inPolarCoords, double a, double b, double rotation) {
    double angle;
    double wheelSpeed;
    double driveSpeedNoTurn;
    if (inPolarCoords) {
      wheelSpeed = Math.max(Math.abs(a), Math.abs(rotation));
      angle = b;
      driveSpeedNoTurn = a;
    } else {
      double square = Math.sqrt(a * a + b * b);
      driveSpeedNoTurn = square / (square / Math.max(Math.abs(a), Math.abs(b)));
      wheelSpeed = square / (square / Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.abs(rotation)));
      angle = Units.radiansToDegrees(Math.atan2(a, b));
    }

    if (navx != null) {
      angle -= navx.getAngle();
    }
    // Reorder wheels based on angle
    SwerveModule[] wheels = new SwerveModule[4];
    double unwrapped = unwrap(angle);
    if (unwrapped < -135 || unwrapped > 135) {
      // left
      wheels[0] = modules[2];
      wheels[1] = modules[3];
      wheels[2] = modules[0];
      wheels[3] = modules[1];
    } else if (unwrapped < -45) {
      // bottom
      wheels[0] = modules[1];
      wheels[1] = modules[2];
      wheels[2] = modules[3];
      wheels[3] = modules[0];
    } else if (unwrapped < 45) {
      // right
      wheels[0] = modules[0];
      wheels[1] = modules[1];
      wheels[2] = modules[2];
      wheels[3] = modules[3];
    } else {
      // top
      wheels[0] = modules[3];
      wheels[1] = modules[0];
      wheels[2] = modules[1];
      wheels[3] = modules[2];
    }

    for (int i = 0; i < 4; i++) {
      // reverse rotate wheels behind
      double turnModifier = i % 2 == 0 ? 1 : -1;
      // double speedModifier = driveSpeedNoTurn;
      turnModifier *= rotation * speedTurnAngle * driveSpeedNoTurn;
      double _angle = angle + turnModifier;

      wheels[i].setTurnPosition(_angle);
      wheels[i].setPower(wheelSpeed);
    }

    SmartDashboard.putNumber("Drive Speed", driveSpeedNoTurn);
    SmartDashboard.putNumber("Turn Speed", rotation);
    SmartDashboard.putNumber("Angle", angle);
  }

  public void testSingleWheel(int index, double power, double angle) {
    modules[index].setPower(power);
    modules[index].setTurnPosition(angle);
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

  private double unwrap(double angle) {
    while (angle > 180) {
      angle -= 360;
    }
    while (angle < -180) {
      angle += 360;
    }
    return angle;
  }
}
