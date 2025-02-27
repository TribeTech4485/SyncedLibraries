package frc.robot.SyncedLibraries.SystemBases.Utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Limits the change in a 2d input to a fixed amount per second
 * See {@link SlewRateLimiter} for more information
 */
public class SlewLimiter2d {
  Translation2d previousPos;
  double maxChangePerSecond;

  /**
   * See {@link SlewRateLimiter} for more information
   * 
   * @param limit The maximum change per second allowed for the position.
   */
  public SlewLimiter2d(double limit) {
    previousPos = new Translation2d(0, 0);
    maxChangePerSecond = limit;
  }

  /**
   * Calculates the new position based on the input coordinates (x, y) while
   * limiting the rate of change.
   *
   * @param x The x-coordinate of the current position.
   * @param y The y-coordinate of the current position.
   * @return A double array containing the new x and y coordinates after applying
   *         the slew rate limiter.
   */
  public double[] calculate(double x, double y) {
    Translation2d currentPos = new Translation2d(x, y);
    Translation2d delta = currentPos.minus(previousPos);
    double distance = delta.getNorm();
    if (distance > maxChangePerSecond * 0.02) {
      delta = delta.times(maxChangePerSecond * 0.02 / distance);
    }
    previousPos = previousPos.plus(delta);
    return new double[] { previousPos.getX(), previousPos.getY() };
  }

  public void setSlewLimit(double limit) {
    maxChangePerSecond = limit;
  }
}
