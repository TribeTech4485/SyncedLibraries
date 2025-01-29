package frc.robot.SyncedLibraries.SystemBases.PathPlanning;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

import java.util.ArrayList;
import java.util.List;

public class HolonomicDriveBase {
  protected final HolonomicDriveController holoController;
  protected final SwerveDriveBase driveBase;

  public HolonomicDriveBase(SwerveDriveBase driveBase) {
    holoController = new HolonomicDriveController(
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)));
    this.driveBase = driveBase;
  }

  public ChassisSpeeds getOutput() {
    SwerveDriveOdometry odometry = driveBase.getOdometry();
    List<Translation2d> waypoints = new ArrayList<>();
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        waypoints,
        new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
        new TrajectoryConfig(1, 1));
    Trajectory.State goal = trajectory.sample(1);
    return holoController.calculate(odometry.getPoseMeters(), goal, 
    trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation());
  }
}
