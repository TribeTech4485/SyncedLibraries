package frc.robot.SyncedLibraries.SystemBases.PathPlanning;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.BackgroundTrajectoryGenerator;

public class TrajectoryMoveCommand extends Command {
  protected final BackgroundTrajectoryGenerator generator;
  protected final SwerveDriveBase driveBase;
  protected final boolean relativeToInitialPose;
  protected double stopTimeMultiplier = 1;
  protected final HolonomicDriveController holoDrive;
  protected SwerveControllerCommand swerveControllerCommand;

  public TrajectoryMoveCommand(BackgroundTrajectoryGenerator generator, HolonomicDriveController holoController,
      SwerveDriveBase swerveDriveBase, boolean relativeToInitialPose) {
    this.generator = generator;
    this.driveBase = swerveDriveBase;
    this.holoDrive = holoController;
    this.relativeToInitialPose = relativeToInitialPose;
  }

  @Override
  public void initialize() {
    swerveControllerCommand = null;
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
    new RunCommand(() -> driveBase.stop(), driveBase).withTimeout(
        driveBase.maxSpeed.in(MetersPerSecond) / driveBase.maxAcceleration.in(MetersPerSecondPerSecond)
            * stopTimeMultiplier)
        .schedule();
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
        trajectory = trajectory
            .relativeTo(driveBase.getOdometry().getPoseMeters());
      }

      swerveControllerCommand = new SwerveControllerCommand(
          trajectory,
          () -> new Pose2d(driveBase.getOdometry().getPoseMeters().getTranslation(),
              driveBase.getGyroAngle()),
          driveBase.getKinematics(), holoDrive,
          (states) -> driveBase.inputDrivingSpeeds(driveBase.getKinematics().toChassisSpeeds(states), -1),
          driveBase);
    }
  }

  public TrajectoryMoveCommand setStopTimeMultiplier(double stopTimeMultiplier) {
    this.stopTimeMultiplier = stopTimeMultiplier;
    return this;
  }
}
