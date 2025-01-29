package frc.robot.SyncedLibraries.SystemBases.PathPlanning;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Subsystems.Swerve.Drivetrain;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Testing {
  TrajectoryConfig config = new TrajectoryConfig(1, 1);
  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, 0)),
      new Pose2d(3, 0, new Rotation2d(0)),
      config);

  public void test() {
    Drivetrain drivetrain = new Drivetrain();
    HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(0, 0, 0), new PIDController(0, 0, 0), new ProfiledPIDController(0, 0, 0, null));
    drivetrain.inputDrivingSpeeds(controller.calculate(null, trajectory.sample(1), null), 0);
  }
}
