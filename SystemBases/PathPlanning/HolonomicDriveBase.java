package frc.robot.SyncedLibraries.SystemBases.PathPlanning;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

public class HolonomicDriveBase {
  public final HolonomicDriveController holoController;
  public final SwerveDriveBase driveBase;

  public HolonomicDriveBase(SwerveDriveBase driveBase, PIDConfig xConfig, PIDConfig yConfig, PIDConfig rotConfig) {
    this.holoController = new HolonomicDriveController(
        new PIDController(xConfig.P, xConfig.I, xConfig.D),
        new PIDController(yConfig.P, yConfig.I, yConfig.D),
        driveBase.getTurnController());
    this.driveBase = driveBase;
  }

  public ChassisSpeeds getOutput(Trajectory trajectory, Pose2d currentPose, double currentTime) {
    State desired = trajectory.sample(currentTime);
    return holoController.calculate(
        currentPose, desired.poseMeters,
        desired.velocityMetersPerSecond,
        desired.poseMeters.getRotation());
  }
}
