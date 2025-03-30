package frc.robot.SyncedLibraries.SystemBases.PathPlanning;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

public class DistanceMoveCommand extends Command {
  SwerveDriveBase drivetrain;
  PIDConfig xConfig;
  PIDConfig yConfig;
  PIDConfig turnConfig;
  Distance xDistance;
  Distance yDistance;
  Rotation2d startAngle;
  ProfiledPIDController xController;
  ProfiledPIDController yController;
  PIDController turnController;
  Pose2d startingPose;

  public DistanceMoveCommand(SwerveDriveBase drivetrain, Distance xDistance, Distance yDistance,
      PIDConfig xConfig, PIDConfig yConfig, PIDConfig turnConfig) {
    this.drivetrain = drivetrain;
    this.xDistance = xDistance;
    this.yDistance = yDistance.unaryMinus();
    this.xConfig = xConfig;
    this.yConfig = yConfig;
    this.turnConfig = turnConfig;
    // addRequirements(drivetrain);
    // SmartDashboard.putData("AAAAAA MoveDistanceCommand", this);
  }

  @Override
  public void initialize() {
    xController = new ProfiledPIDController(xConfig.P, xConfig.I, xConfig.D, new TrapezoidProfile.Constraints(
        xConfig.maxLinearVelocity.in(MetersPerSecond), xConfig.maxLinearAcceleration.in(MetersPerSecondPerSecond)));
    yController = new ProfiledPIDController(yConfig.P, yConfig.I, yConfig.D, new TrapezoidProfile.Constraints(
        yConfig.maxLinearVelocity.in(MetersPerSecond), yConfig.maxLinearAcceleration.in(MetersPerSecondPerSecond)));
    xController.setGoal(xDistance.in(Meters));
    yController.setGoal(yDistance.in(Meters));
    xController.setTolerance(0.005);
    yController.setTolerance(0.005);
    turnController = new PIDController(turnConfig.P, turnConfig.I, turnConfig.D);
    startingPose = drivetrain.getOdometry().getPoseMeters();
    startAngle = drivetrain.getGyroAngle();
    drivetrain.getTurnController().reset(drivetrain.getGyroAngle().getRadians());

    System.out.println("Turning by " + turnController.getSetpoint() + " radians, moving to "
        + xDistance.in(Feet) + "ft, " + yDistance.in(Feet) + "ft");
  }

  @Override
  public void execute() {
    Transform2d traveledTransform = drivetrain.getOdometry().getPoseMeters().minus(startingPose);
    // SmartDashboard.putNumber("AAA has traveled X", traveledTransform.getX());
    // SmartDashboard.putNumber("AAA has traveled Y", traveledTransform.getY());
    double xOutput = xController.calculate(traveledTransform.getX());
    double yOutput = yController.calculate(traveledTransform.getY());
    double turnOutput = turnController
        .calculate(drivetrain.getGyroAngle().minus(startAngle).getRadians());
    if (!drivetrain.getGyro().isConnected()) {
      turnOutput = 0;
    }
    // SmartDashboard.putNumber("AAA xOutput", xOutput);
    // SmartDashboard.putNumber("AAA yOutput", yOutput);
    // SmartDashboard.putNumber("AAA xError", xController.getPositionError());
    // SmartDashboard.putNumber("AAA yError", yController.getPositionError());
    // SmartDashboard.putNumber("AAA xGoal", xController.getGoal().position);
    // SmartDashboard.putNumber("AAA yGoal", yController.getGoal().position);
    // SmartDashboard.putNumber("AAA xSetpoint",
    // xController.getSetpoint().position);
    // SmartDashboard.putNumber("AAA ySetpoint",
    // yController.getSetpoint().position);
    drivetrain.setFieldRelative(false);
    drivetrain.inputDrivingX_Y(MetersPerSecond.of(-xOutput - xController.getSetpoint().velocity),
        MetersPerSecond.of(-yOutput - yController.getSetpoint().velocity),
        RadiansPerSecond.of(turnOutput), -1);
  }

  @Override
  public boolean isFinished() {
    try {
      if (xDistance.compareTo(Inches.of(0.25)) < 0 && yDistance.compareTo(Inches.of(0.25)) < 0) {
        System.out.println("Move finished due to distance");
        return true;
      }
      ChassisSpeeds speeds = drivetrain.getLiveChassisSpeed();
      if (xController.atGoal() && yController.atGoal()) {
        System.out.println("Finished move succesfully");
        return true;
      }
      if (Math.hypot(speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond) > xConfig.maxLinearVelocity.in(MetersPerSecond) * 5) {
        DriverStation.reportError("Finished move due to speed", false);
        return true;
      }
    } catch (Exception e) {
    }

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Move ended with interrupted: " + interrupted);
    new RunCommand(() -> drivetrain.stop()).withTimeout(.25).schedule();

  }
}
