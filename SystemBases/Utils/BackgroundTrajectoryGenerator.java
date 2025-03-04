package frc.robot.SyncedLibraries.SystemBases.Utils;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Supplier;
import edu.wpi.first.math.trajectory.Trajectory;

public class BackgroundTrajectoryGenerator {
  private final Future<Trajectory> future;
  private Trajectory result = null;
  private boolean isCompleted = false;

  /**
   * A class that generates a trajectory in the background using a provided
   * function.
   * The trajectory generation is performed in a separate thread to avoid blocking
   * the main thread as trajectory generation can be computationally expensive.
   * <p>
   * Input a lambda function that returns a trajectory.
   * To check if the generation is completed, use {@link #isDone()}, then use
   * {@link #getTrajectory()} to retrieve the result.
   *
   * @param function A Supplier that provides the trajectory to be generated.
   */
  public BackgroundTrajectoryGenerator(Supplier<Trajectory> function) {
    ExecutorService executor = Executors.newSingleThreadExecutor();
    this.future = executor.submit(() -> {
      Trajectory value = function.get();
      synchronized (this) {
        result = value;
        isCompleted = true;
      }
      return value;
    });
    executor.shutdown();
  }

  /**
   * Code halts until the generation is completed, use the {@link #isDone()} to
   * check if done, then call this function to retrieve the result.
   * <p>
   * <strong>Warning: returns null if the function errors out</strong>
   */
  public synchronized Trajectory getTrajectory() {
    if (!isCompleted) {
      try {
        result = future.get(); // Blocks if not completed
      } catch (Exception e) {
        System.out.println("Error getting trajectory!! " + e.getMessage());
        e.printStackTrace();
        result = null;
      }
      isCompleted = true;
    }
    return result;
  }

  public boolean isDone() {
    return isCompleted || future.isDone();
  }
}
