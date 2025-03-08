package frc.robot.SyncedLibraries.SystemBases.PathPlanning;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

/**
 * A class that wraps a SwerveDriveBase and a HolonomicDriveController to
 * provide a simple holonomic
 * drive method for following trajectories.
 */
public class HolonomicDriveBase {
  public final HolonomicDriveController holoController;
  public final SwerveDriveBase driveBase;

  public HolonomicDriveBase(SwerveDriveBase driveBase, PIDConfig xConfig, PIDConfig yConfig) {
    this.holoController = new HolonomicDriveController(
        new PIDController(xConfig.P, xConfig.I, xConfig.D),
        new PIDController(yConfig.P, yConfig.I, yConfig.D),
        driveBase.getTurnController());
    this.driveBase = driveBase;
  }

  public ChassisSpeeds getOutput(State desiredState, Pose2d currentPose) {
    return holoController.calculate(
        currentPose, desiredState.poseMeters,
        desiredState.velocityMetersPerSecond,
        desiredState.poseMeters.getRotation());
  }

  public void drive(State desiredState, Pose2d currentPose) {
    driveBase.inputDrivingSpeeds(getOutput(desiredState, currentPose), -1);
  }
}
