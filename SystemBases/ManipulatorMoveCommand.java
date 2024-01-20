package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;

public class ManipulatorMoveCommand extends Command {
  private ManipulatorBase manipulator;
  private double position;
  private double tolerance;
  private PIDController pid;

  public ManipulatorMoveCommand(ManipulatorBase manipulator, double position, double tolerance, double kP, double kI, double kD) {
    this.manipulator = manipulator;
    this.position = position;
    this.tolerance = tolerance;
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(tolerance);
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.stop();
    pid.reset();
    pid.setSetpoint(position);
    if (position == Integer.MAX_VALUE) {
      // https://www.youtube.com/watch?v=dQw4w9WgXcQ //
      DriverStation.reportError("ManipulatorMoveCommand: position not set", true);
      CommandScheduler.getInstance().cancel(this);
    }
  }

  @Override
  public void execute() {
    manipulator.setPower(pid.calculate(manipulator.getPosition(), position));
  }

  @Override
  public void end(boolean interrupted) {
    manipulator.stop();
    if (interrupted) {
      DriverStation.reportWarning("ManipulatorMoveCommand: interrupted", false);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(manipulator.getPosition() - position) < tolerance;
  }

  public void setTargetPosition(double position) {
    this.position = position;
  }

  public double getTargetPosition() {
    return position;
  }
}
