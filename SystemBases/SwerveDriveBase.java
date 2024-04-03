// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SyncedLibraries.SystemBases;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
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
   * Pi is reverse
   * In radians
   */

  @Override
  public void periodic() {
  }

  private class SwerveModule extends SubsystemBase {
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder turnEncoder;
    private RelativeEncoder driveEncoder;
    private SparkPIDController turnPID;
    private SparkPIDController drivePID;
    private double targetTurnPosition;
    private double targetDriveSpeed;
    private boolean powerMode = false;

    public SwerveModule(CANSparkMax driveMotor, CANSparkMax turnMotor, double maxDriveSpeed,
        double turnP, double turnI, double turnD,
        double driveP, double driveI, double driveD) {
      this.driveMotor = driveMotor;
      this.turnMotor = turnMotor;
      this.turnEncoder = turnMotor.getEncoder();

      turnPID = turnMotor.getPIDController();
      turnPID.setP(turnP);
      turnPID.setI(turnI);
      turnPID.setD(turnD);
      turnPID.setPositionPIDWrappingEnabled(true);
      turnPID.setPositionPIDWrappingMinInput(-Math.PI);
      turnPID.setPositionPIDWrappingMaxInput(Math.PI);

      drivePID = driveMotor.getPIDController();
      drivePID.setP(driveP);
      drivePID.setI(driveI);
      drivePID.setD(driveD);
      setMaxDriveSpeed(maxDriveSpeed);
    }

    public void setMaxDriveSpeed(double speed) {
      drivePID.setOutputRange(-speed, speed);
      drivePID.setSmartMotionMaxVelocity(speed, 0);
      drivePID.setIZone(speed * 0.1);
    }

    /** Burn flash on all motor controllers */
    public void burn() {
      turnMotor.burnFlash();
      driveMotor.burnFlash();
    }

    /** Set speed from in rpm */
    public void setDriveSpeed(double speed) {
      targetDriveSpeed = speed;
    }

    /** Set position in radians */
    public void setTurnPosition(double position) {
      targetTurnPosition = position;
    }

    /** Returns in radians */
    public double getTargetTurnPosition() {
      return targetTurnPosition;
    }

    /** Returns in rpm */
    public double getTargetDriveSpeed() {
      return targetDriveSpeed;
    }

    /** Returns in radians */
    public double getTurnPosition() {
      return turnEncoder.getPosition();
    }

    /** Returns in rpm */
    public double getDriveSpeed() {
      return driveMotor.getEncoder().getVelocity();
    }

    public RelativeEncoder getTurnEncoder() {
      return turnEncoder;
    }

    /**
     * Use for pushing or speed, will likely
     * turn uncontrollably when driving
     */
    public void setPowerMode(boolean powerMode) {
      this.powerMode = powerMode;
    }

    @Override
    public void periodic() {
      /*
       * Find closest path to target angle
       * If error is greater than 90 degrees,
       * then face opposite direction
       * Reversing drive motor is handled by the cosine function in the drivePID
       */
      double error = Math.abs(unWrapError());
      if (error > Math.PI / 2) {
        targetTurnPosition += Math.PI;
      }
      turnPID.setReference(targetTurnPosition, ControlType.kSmartMotion);

      double cosErr = Math.cos(unWrapError());
      if (Math.abs(cosErr) < 0.1) {
        cosErr = 0;
      }

      if (powerMode) {
        // Go full power
        drivePID.setReference(
            cosErr, ControlType.kDutyCycle);
      } else {
        // Actually be smart about speed
        drivePID.setReference(cosErr * targetDriveSpeed,
            ControlType.kSmartVelocity);
      }
    }

    /** If value is greater than Pi, it will be wrapped to -Pi */
    private double unWrapError() {
      double error = getTargetTurnPosition() - getTurnPosition();
      if (error > Math.PI) {
        error -= 2 * Math.PI;
      } else if (error < -Math.PI) {
        error += 2 * Math.PI;
      }
      return error;
    }
  }
}
