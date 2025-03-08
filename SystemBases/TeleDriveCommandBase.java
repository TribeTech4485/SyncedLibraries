package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

public abstract class TeleDriveCommandBase extends Command {
  private boolean haveTriggersBeenBound = false;
  protected final ControllerBase[] controllers = new ControllerBase[5];
  protected double deadBand = 0.1;
  protected DriveModes driveMode = DriveModes.DESIRED_ANGLE;
  protected SwerveDriveBase swerveTrain;
  protected boolean x_locked = false;
  protected boolean allowTurn = true;
  protected boolean usePOV = false;

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
          if (controllers[0].isXbox || controllers[0].isPS4) {
            if (Math.abs(controllers[0].getLeftX()) <= deadBand && Math.abs(controllers[0].getLeftY()) <= deadBand) {
              // If the joystick is not being used, don't rotate
              swerveTrain.inputDrivingX_Y(controllers[0].getRightY(), controllers[0].getRightX(),
                  0, usePOV ? controllers[0].getPOV() : -1);
            } else {
              swerveTrain.inputDrivingX_Y_A(controllers[0].getRightY(), controllers[0].getRightX(),
                  new Rotation2d(-controllers[0].getLeftY(), controllers[0].getLeftX()),
                  usePOV ? controllers[0].getPOV() : -1);

            }
          } else if (controllers[0].isJoystick) {
            swerveTrain.inputDrivingX_Y_A(controllers[0].getLeftX(), controllers[0].getLeftY(),
                new Rotation2d(Units.degreesToRadians(controllers[0].getPOV())), -1);
          }
          break;

        case ROTATION_SPEED:
          if (controllers[0].isXbox || controllers[0].isPS4) {
            swerveTrain.inputDrivingX_Y(controllers[0].getRightY(), controllers[0].getRightX(),
                -controllers[0].getLeftX(),
                usePOV ? controllers[0].getPOV() : -1);
          } else if (controllers[0].isJoystick) {
            swerveTrain.inputDrivingX_Y(controllers[0].getLeftY(), controllers[0].getLeftX(),
                allowTurn ? -controllers[0].getRightX() : 0,
                usePOV ? controllers[0].getPOV() : -1);
          }
          break;
      }
    }

    SmartDashboard.putString("Driving mode", driveMode.toString());
  }

  /**
   * Keybinds that will be synchronized between all robot drivers/years
   * <p>
   * Call this in RobotContainer.configureBindings() for readability
   */
  public void setNormalTriggerBinds() {
    // Leave ABXY and POV for year specific commands
    if (haveTriggersBeenBound) {
      return;
    }
    haveTriggersBeenBound = true;

    if (controllers[0].isXbox || controllers[0].isPS4) {
      if (false) {
        // Using normal controllers
        controllers[0].LeftStickPress.and(controllers[0].RightStickPress.negate())
            .onTrue(new InstantCommand(() -> swerveTrain.setSudoMode(true)));
        controllers[0].RightStickPress.and(controllers[0].LeftStickPress.negate())
            .onTrue(new InstantCommand(() -> swerveTrain.setSudoMode(false)));

        controllers[0].LeftStickPress.and(controllers[0].RightStickPress)
            .onChange(new InstantCommand(() -> swerveTrain.setSudoMode(false)))
            .onTrue(new InstantCommand(swerveTrain::enableXLock))
            .onTrue(new InstantCommand(() -> x_locked = true))
            .onFalse(new InstantCommand(swerveTrain::disableXLock))
            .onFalse(new InstantCommand(() -> x_locked = false));

        controllers[0].RightTrigger
            .onTrue(new InstantCommand(() -> swerveTrain.setBrakeMode(true)))
            .onFalse(new InstantCommand(() -> swerveTrain.setBrakeMode(false)));

        controllers[0].LeftTrigger
            .onTrue(new InstantCommand(() -> swerveTrain.setSlowMode(true)))
            .onFalse(new InstantCommand(() -> swerveTrain.setSlowMode(false)));

        // the right one
        controllers[0].Start.onTrue(new InstantCommand(swerveTrain::resetGyro));

        // the left one
        controllers[0].Options.onTrue(new InstantCommand(() -> {
          // Cancel all other commands using the drive train
          if (swerveTrain.getCurrentCommand() != this) {
            swerveTrain.getCurrentCommand().cancel();
          }
        }));

        controllers[0].RightBumper
            // .onTrue(new InstantCommand(() -> driveMode = DriveModes.DESIRED_ANGLE))
            .onTrue(new InstantCommand(() -> swerveTrain.setFieldRelative(true)));
        controllers[0].LeftBumper
            // .onTrue(new InstantCommand(() -> driveMode = DriveModes.ROTATION_SPEED))
            .onTrue(new InstantCommand(() -> swerveTrain.setFieldRelative(false)));
      } else {
        controllers[0].LeftStickPress
            .onChange(new InstantCommand(() -> swerveTrain.setSlowMode(false)));
        controllers[0].RightStickPress
            .onChange(new InstantCommand(() -> swerveTrain.setSlowMode(true)));
        controllers[0].LeftStickPress.and(controllers[0].RightStickPress)
            // .onChange(new InstantCommand(() -> swerveTrain.setSudoMode(false)))
            .onTrue(new InstantCommand(swerveTrain::enableXLock))
            .onTrue(new InstantCommand(() -> x_locked = true));
        controllers[0].LeftStickPress.and(controllers[0].RightStickPress)
            .and(controllers[0].LeftTrigger.negate()).and(controllers[0].RightTrigger.negate())
            .onFalse(new InstantCommand(swerveTrain::disableXLock))
            .onFalse(new InstantCommand(() -> x_locked = false));

        controllers[0].RightTrigger
            .onChange(new InstantCommand(() -> swerveTrain.setFieldRelative(true)))
            // controllers[0].RightBumper
            .onChange(new InstantCommand(() -> driveMode = DriveModes.DESIRED_ANGLE));
        controllers[0].LeftTrigger
            .onChange(new InstantCommand(() -> swerveTrain.setFieldRelative(false)))
            // controllers[0].LeftBumper
            .onChange(new InstantCommand(() -> driveMode = DriveModes.ROTATION_SPEED));

        controllers[0].LeftTrigger
            .and(controllers[0].RightTrigger)
            .and(controllers[0].RightBumper)
            .and(controllers[0].LeftBumper)
            .onTrue(new InstantCommand(swerveTrain::resetGyro));
      }

    } else if (controllers[0].isJoystick) {
      // Using flight sticks
      controllers[0].buttons[2]
          .onTrue(new InstantCommand(() -> swerveTrain.setSlowMode(true)))
          .onFalse(new InstantCommand(() -> swerveTrain.setSlowMode(false)));

      controllers[0].buttons[1]
          .onTrue(new InstantCommand(() -> allowTurn = true))
          .onFalse(new InstantCommand(() -> allowTurn = false));
      // .onTrue(new InstantCommand(() -> swerveTrain.setSudoMode(true)))
      // .onFalse(new InstantCommand(() -> swerveTrain.setSudoMode(false)));

      controllers[0].buttons[5]
          .onChange(new InstantCommand(() -> swerveTrain.setFieldRelative(true)));
      controllers[0].buttons[3]
          .onChange(new InstantCommand(() -> swerveTrain.setFieldRelative(false)));
      controllers[0].buttons[3].and(controllers[0].buttons[5])
          .onTrue(new InstantCommand(swerveTrain::resetGyro));

      controllers[0].buttons[6]
          .onTrue(new InstantCommand(swerveTrain::enableXLock))
          .onTrue(new InstantCommand(() -> x_locked = true))
          .onFalse(new InstantCommand(swerveTrain::disableXLock))
          .onFalse(new InstantCommand(() -> x_locked = false));
      controllers[0].buttons[9]
          .onTrue(new InstantCommand(swerveTrain::enableXLock))
          .onTrue(new InstantCommand(() -> x_locked = true));

      controllers[0].buttons[7]
          .onTrue(new InstantCommand(() -> swerveTrain.setBrakeMode(true)));
      controllers[0].buttons[8]
          .onTrue(new InstantCommand(() -> swerveTrain.setBrakeMode(false)));
    }
  }

  protected enum DriveModes {
    DESIRED_ANGLE, ROTATION_SPEED
  }
}
