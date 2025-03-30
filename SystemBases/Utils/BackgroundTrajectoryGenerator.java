package frc.robot.SyncedLibraries.SystemBases.Utils;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

/**
 * It is stated that trajectory generation takes 10-20ms, which is close enough
 * to the periodic timing (20ms)
 * that it is recommended to generate trajectories in a separate thread to avoid
 * blocking the main thread.
 */
public class BackgroundTrajectoryGenerator {
  private final Future<Trajectory> future;
  private Trajectory trajectory;

  /**
   * A class that generates a trajectory in the background using a provided
   * function.
   * The trajectory generation is performed in a separate thread to avoid blocking
   * the main thread. See inside for why.
   * <p>
   * Input a lambda function that returns a trajectory.
   * To check if the generation is completed, use {@link #isDone()}, then use
   * {@link #getTrajectory()} to retrieve the result.
   *
   * @param startPos          The starting position of the trajectory
   * @param endPos            The ending position of the trajectory
   * @param interiorWaypoints The interior waypoints of the trajectory
   * @param driveBase         The swerve drive base to use for max speed and max
   *                          acceleration
   * @param margin            The margin to multiply the max speed and max
   *                          acceleration by
   */
  public BackgroundTrajectoryGenerator(Pose2d startPos, Pose2d endPos, List<Translation2d> interiorWaypoints,
      SwerveDriveBase driveBase, double margin) {
    this(startPos, endPos, interiorWaypoints,
        driveBase.maxSpeed.times(margin),
        driveBase.maxAcceleration.times(margin));
  }

  /**
   * A class that generates a trajectory in the background using a provided
   * function.
   * The trajectory generation is performed in a separate thread to avoid blocking
   * the main thread. See inside for why.
   * <p>
   * Input a lambda function that returns a trajectory.
   * To check if the generation is completed, use {@link #isDone()}, then use
   * {@link #getTrajectory()} to retrieve the result.
   *
   * @param startPos          The starting position of the trajectory
   * @param endPos            The ending position of the trajectory
   * @param interiorWaypoints The interior waypoints of the trajectory
   * @param maxSpeed          The maximum speed of the robot
   * @param maxAcceleration   The maximum acceleration of the robot
   */
  public BackgroundTrajectoryGenerator(Pose2d startPos, Pose2d endPos, List<Translation2d> interiorWaypoints,
      LinearVelocity maxSpeed, LinearAcceleration maxAcceleration) {
    TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAcceleration);

    ExecutorService executor = Executors.newSingleThreadExecutor();
    this.future = executor.submit(() -> TrajectoryGenerator.generateTrajectory(
        startPos, interiorWaypoints, endPos, config));
    executor.shutdown();
    System.out.println("Generating trajectory from " + startPos + " to " + endPos);
  }

  /**
   * Code halts until the generation is completed, use the {@link #isDone()} to
   * check if done, then call this function to retrieve the result.
   * <p>
   * <strong>Warning: returns null if the function errors out</strong>
   */
  public synchronized Trajectory getTrajectory() {
    try {
      return future.get(); // Waits if not completed
    } catch (Exception e) {
      System.out.println("Error getting trajectory!! " + e.getMessage());
      e.printStackTrace();
      return null;
    }
  }

  public boolean isDone() {
    return future.isDone();
  }
}
