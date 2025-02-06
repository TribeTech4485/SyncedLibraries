package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

public abstract class TeleDriveCommandBase extends Command {
  // TODO: Add swerve drive support
  protected final ControllerBase[] controllers = new ControllerBase[3];
  protected double deadBand = 0.1;
  protected DriveModes driveMode = DriveModes.ROTATION_SPEED;
  protected SwerveDriveBase swerveTrain;
  protected boolean x_locked = false;
  private boolean haveTriggersBeenBound = false;

  public TeleDriveCommandBase(SwerveDriveBase driveTrain, ControllerBase... controllers) {
    addRequirements(driveTrain);
    // TODO: Add tank drive support
    this.swerveTrain = driveTrain;
    for (int i = 0; i < this.controllers.length; i++) {
      if (controllers.length <= i) {
        this.controllers[i] = new ControllerBase(-1);
        continue;
      }
      this.controllers[i] = controllers[i];
    }
    swerveTrain.setDefaultCommand(this);
  }

  @Override
  public void initialize() {
    // In case if not called in configureBindings()
    setNormalTriggerBinds();
  }

  @Override
  public void execute() {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }
    if (!x_locked) {
      switch (driveMode) {
        case DESIRED_ANGLE:
          if (controllers[0].getRightX() == 0 && controllers[0].getRightY() == 0) {
            // If the joystick is not being used, don't rotate
            swerveTrain.inputDrivingX_Y(controllers[0].getLeftX(), controllers[0].getLeftY(), 0, -1);
          } else {
            swerveTrain.inputDrivingX_Y_A(controllers[0].getLeftX(), controllers[0].getLeftY(),
                new Rotation2d(controllers[0].getRightX(), controllers[0].getRightY()), -1);
          }
          break;

        case ROTATION_SPEED:
          swerveTrain.inputDrivingX_Y(controllers[0].getLeftX(), controllers[0].getLeftY(), controllers[0].getRightX());
          break;
      }
    }

    SmartDashboard.putString("Driving mode", driveMode.toString());
  }

  /**
   * Keybinds that will be synchronized between all robot drivers
   * <p>
   * Call this in RobotContainer.configureBindings() for readability
   */
  public void setNormalTriggerBinds() {
    // Leave ABXY and POV for year specific commands
    if (haveTriggersBeenBound) {
      return;
    }
    haveTriggersBeenBound = true;

    controllers[0].LeftStickPress.onTrue(new InstantCommand(() -> swerveTrain.setSudoMode(true)));
    controllers[0].RightStickPress.onTrue(new InstantCommand(() -> swerveTrain.setSudoMode(false)));

    controllers[0].LeftStickPress.and(controllers[0].RightStickPress)
        .onTrue(new InstantCommand(swerveTrain::enableXLock))
        .onTrue(new InstantCommand(() -> swerveTrain.setSudoMode(false)))
        .onTrue(new InstantCommand(() -> x_locked = true))
        .onFalse(new InstantCommand(swerveTrain::disableXLock))
        .onFalse(new InstantCommand(() -> x_locked = false));

    controllers[0].RightTrigger.onTrue(new InstantCommand(() -> swerveTrain.setBrakeMode(true)));
    controllers[0].RightTrigger.onFalse(new InstantCommand(() -> swerveTrain.setBrakeMode(false)));

    controllers[0].LeftTrigger.onTrue(new InstantCommand(() -> swerveTrain.setSlowMode(true)));
    controllers[0].LeftTrigger.onFalse(new InstantCommand(() -> swerveTrain.setSlowMode(false)));

    // the right one
    controllers[0].Start.onTrue(new InstantCommand(swerveTrain::resetGyro));
    // the left one
    controllers[0].Options.onTrue(new InstantCommand(() -> {
      // Cancel all other commands using the drive train
      if (swerveTrain.getCurrentCommand() != this) {
        swerveTrain.getCurrentCommand().cancel();
      }
    }));

    controllers[0].RightBumper.onTrue(new InstantCommand(() -> driveMode = DriveModes.DESIRED_ANGLE))
        .onTrue(new InstantCommand(() -> swerveTrain.setFieldRelative(true)));
    controllers[0].LeftBumper.onTrue(new InstantCommand(() -> driveMode = DriveModes.ROTATION_SPEED))
        .onTrue(new InstantCommand(() -> swerveTrain.setFieldRelative(false)));
  }

  protected enum DriveModes {
    DESIRED_ANGLE, ROTATION_SPEED
  }
}
