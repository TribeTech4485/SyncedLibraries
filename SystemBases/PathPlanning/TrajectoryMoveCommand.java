package frc.robot.SyncedLibraries.SystemBases.PathPlanning;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.BackgroundTrajectoryGenerator;

public class TrajectoryMoveCommand extends Command {
  protected final BackgroundTrajectoryGenerator generator;
  protected final SwerveDriveBase driveBase;
  protected final boolean relativeToInitialPose;
  protected final HolonomicDriveController holoDrive;
  protected SwerveControllerCommand swerveControllerCommand;

  public TrajectoryMoveCommand(BackgroundTrajectoryGenerator generator, HolonomicDriveController holoController,
      SwerveDriveBase swerveDriveBase, boolean relativeToInitialPose) {
    this.generator = generator;
    this.driveBase = swerveDriveBase;
    this.holoDrive = holoController;
    this.relativeToInitialPose = relativeToInitialPose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (swerveControllerCommand == null) {
      retrieveTrajectory();
    }
    if (swerveControllerCommand != null) {
      if (!swerveControllerCommand.isScheduled()) {
        swerveControllerCommand.schedule();
      }
    } else {
      driveBase.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (swerveControllerCommand == null) {
      return false;
    } else {
      return swerveControllerCommand.isFinished();
    }
  }

  private void retrieveTrajectory() {
    if (swerveControllerCommand == null && generator.isDone()) {
      Trajectory trajectory = generator.getTrajectory();
      if (relativeToInitialPose) {
        trajectory = trajectory.relativeTo(driveBase.getOdometry().getPoseMeters());
      }

      swerveControllerCommand = new SwerveControllerCommand(
          trajectory, driveBase.getOdometry()::getPoseMeters, driveBase.getKinematics(), holoDrive,
          (states) -> driveBase.inputDrivingSpeeds(driveBase.getKinematics().toChassisSpeeds(states), -1),
          driveBase);
    }
  }
}
