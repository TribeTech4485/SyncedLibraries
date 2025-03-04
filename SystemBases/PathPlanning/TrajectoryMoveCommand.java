package frc.robot.SyncedLibraries.SystemBases.PathPlanning;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SyncedLibraries.SystemBases.Utils.BackgroundTrajectoryGenerator;

public class TrajectoryMoveCommand extends Command {
  private Trajectory trajectory;
  private final BackgroundTrajectoryGenerator generator;
  private final HolonomicDriveBase holoDrive;
  Timer timer = new Timer();

  public TrajectoryMoveCommand(BackgroundTrajectoryGenerator generator, HolonomicDriveBase driveBase) {
    this.generator = generator;
    this.holoDrive = driveBase;
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
      holoDrive.getOutput(trajectory,
          holoDrive.driveBase.getOdometry().getPoseMeters(),
          timer.get());
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
      // start the timer as soon as the trajectory is retrieved as
      // this is when we start following it
      timer.reset();
      timer.start();
    }
  }
}
