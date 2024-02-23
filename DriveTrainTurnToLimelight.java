package frc.robot.SyncedLibraries;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SyncedLibraries.SystemBases.DriveTrainBase;
import frc.robot.SyncedLibraries.SystemBases.LimelightBase;

public class DriveTrainTurnToLimelight extends Command {
  /** Creates a new DriveTrainTurnToLimelight. */
  double targetAngle = 0;
  double currentAngle = 0;
  double angleDifference = 0;
  PIDController pidController;
  DriveTrainBase driveTrainBase;
  LimelightBase limelightBase;
  public int onTargetCounterStart = 10;
  int onTargetCounter = onTargetCounterStart;
  double tolerance = 0;

  public DriveTrainTurnToLimelight(DriveTrainBase DriveTrain, LimelightBase Limelight, double kP, double kI, double kD, double tolerance) {
    addRequirements(DriveTrain, Limelight);
    pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(tolerance);
    this.tolerance = tolerance;
    driveTrainBase = DriveTrain;
    limelightBase = Limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = limelightBase.getX();
    currentAngle = driveTrainBase.getHeadingAngle();
    angleDifference = targetAngle - currentAngle;
    pidController.setSetpoint(angleDifference);
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = driveTrainBase.getHeadingAngle();
    angleDifference = targetAngle - currentAngle;
    double output = pidController.calculate(angleDifference);
    driveTrainBase.doTankDrive(output, -output);
    if (Math.abs(angleDifference) < tolerance) {
      onTargetCounter--;
    } else {
      onTargetCounter = onTargetCounterStart;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTargetCounter <= 0;
  }
}
