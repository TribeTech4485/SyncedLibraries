package frc.robot.SyncedLibraries.SystemBases.PathPlanning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SyncedLibraries.SystemBases.Utils.BackgroundTrajectoryGenerator;

public class TrajectoryMoveCommand extends Command {
  protected Trajectory trajectory;
  protected final BackgroundTrajectoryGenerator generator;
  protected final HolonomicDriveBase holoDrive;
  protected final Pose2d initialPose;
  protected final Timer timer = new Timer();
  protected final boolean relativeToInitialPose;

  public TrajectoryMoveCommand(BackgroundTrajectoryGenerator generator, HolonomicDriveBase driveBase,
      boolean relativeToInitialPose) {
    this.generator = generator;
    this.holoDrive = driveBase;
    this.initialPose = driveBase.driveBase.getOdometry().getPoseMeters();
    this.relativeToInitialPose = relativeToInitialPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    retrieveTrajectory();
    if (trajectory != null) {
      holoDrive.drive(trajectory.sample(timer.get()),
          holoDrive.driveBase.getOdometry().getPoseMeters());
    } else {
      holoDrive.driveBase.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    holoDrive.driveBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  private void retrieveTrajectory() {
    if (trajectory == null && generator.isDone()) {
      trajectory = generator.getTrajectory();
      if (relativeToInitialPose) {
        trajectory = trajectory.relativeTo(initialPose);
      }

      // start the timer as soon as the trajectory is retrieved as
      // this is when we start following it
      timer.reset();
      timer.start();
    }
  }
}
