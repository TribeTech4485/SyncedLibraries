package frc.robot.SyncedLibraries.SystemBases;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Nearly identical to the ManipulatorMoveCommand, but with FeedForward
 * controllers
 */
public class ManipulatorFFMoveCommand extends ManipulatorMoveCommand {
    protected SimpleMotorFeedforward simpleController;
    protected ElevatorFeedforward elevatorController;
    protected ArmFeedforward armController;

    private ProfiledPIDController pidProfiled;

    /**
     * Just like {@link ManipulatorMoveCommand}, but with a selectable FeedForward
     * 
     * @param manipulator     The manipulator to control
     * @param position        The position to move to
     * @param tolerance       The tolerance for the position
     * @param kP              The proportional gain
     * @param kI              The integral gain
     * @param kD              The derivative gain
     * @param feedForwardType The type of feedforward controller to use
     *                        (Simple, Elevator, Arm)
     * @param kS              The static gain
     * @param kV              The velocity gain
     * @param kG              The gravity gain (leave at 0 if not using Arm or
     *                        Elevator)
     * @param maxVelocity     The maximum velocity of the manipulator
     * @param maxAcceleration The maximum acceleration of the manipulator
     */
    public ManipulatorFFMoveCommand(ManipulatorBase manipulator, double position,
            double tolerance, double kP, double kI, double kD, String feedForwardType,
            double kS, double kV, double kG, double maxVelocity, double maxAcceleration) {
        super(manipulator, position, tolerance, kP, kI, kD);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        pidProfiled = new ProfiledPIDController(kP, kI, kD, constraints);
        pidProfiled.setTolerance(tolerance);

        switch (feedForwardType) {
            case "Simple":
                simpleController = new SimpleMotorFeedforward(kS, kV);
                break;
            case "Elevator":
                elevatorController = new ElevatorFeedforward(kS, kG, kV);
                break;
            case "Arm":
                armController = new ArmFeedforward(kS, kG, kV);
                break;
            default:
                System.out.println("Invalid FeedForward type for manipulator \"" + manipulator.getName() + "\", defaulting to Simple. The options are \"Simple\", \"Elevator\", and \"Arm\"");
                simpleController = new SimpleMotorFeedforward(kS, kV);
                break;
        }
    }

    private double getFF() {
        if (simpleController != null) {
            return simpleController.calculate(pidProfiled.getSetpoint().velocity);
        } else if (elevatorController != null) {
            return elevatorController.calculate(pidProfiled.getSetpoint().velocity);
        } else if (armController != null) {
            return armController.calculate(pidProfiled.getSetpoint().position, pidProfiled.getSetpoint().velocity);
        } else {
            throw new IllegalArgumentException("FeedForward controller not set");
        }
    }

    @Override
    public void initialize() {
        manipulator.stop();
        pidProfiled.reset(new TrapezoidProfile.State(manipulator.getPosition(), manipulator.getCurrentSpeed()));
        pidProfiled.setGoal(position);
    }

    @Override
    public void execute() {
        manipulator.setVoltage(pidProfiled.calculate(manipulator.getPosition()) + getFF(),
                false);
        atPosition = isAtPosition();
    }

}
