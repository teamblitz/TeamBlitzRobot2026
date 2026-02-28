
package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.wrist.WristIOKraken;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;


public class Wrist extends BlitzSubsystem {
    private final WristIO io;

    public Wrist(WristIO io) {
        super("Wrist");

        this.io = io;

        setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
        goal = Optional.empty();
    }

    private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL);

    private Optional<TrapezoidProfile.State> goal;
    private TrapezoidProfile.State setpoint;

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

    // Stores the cruise velocity to pass to Motion Magic, set by followGoal
    private double cruiseVelocity = MAX_VELOCITY;

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        if (goal.isPresent() && DriverStation.isEnabled()) {
            TrapezoidProfile.State future_setpoint =
                    profile.calculate(0.02, setpoint, goal.get());

            // Use setpoint.position (the profiled step), not goal (the final target)
            // Also pass cruiseVelocity so Motion Magic knows how fast to move
            io.setMotionMagic(setpoint.position, cruiseVelocity);

            setpoint = future_setpoint;
        }

        if (DriverStation.isDisabled()) {
            // Reset profile while disabled
            setpoint = new TrapezoidProfile.State(getPosition(), 0);
            goal = Optional.empty();

            // Stop wrist
            io.stop();
        }
    }

    // Moves the wrist up manually while held
    public Command move_up() {
        return startEnd(() -> io.setSpeed(0.3), () -> io.setSpeed(0));
    }

    // Moves the wrist down manually while held (negative = opposite direction)
    public Command move_down() {
        return startEnd(() -> io.setSpeed(-0.3), () -> io.setSpeed(0));
    }

    public Command setSpeed(double speed) {
        return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
    }

    // Moves the wrist to the idle (up) position
    public Command goToIdle() {
        return goToPosition(ZERO_POS, -0.1);
    }

    // Moves the wrist to the down position
    public Command goToDown() {
        return goToPosition(EXTENDED_POS, 0.1);
    }

    // Sends the motor to a position at the given cruise velocity,
    // and waits until the wrist is close enough to the target
    public Command goToPosition(double position, double cruiseVelocity) {
        return followGoal(() -> position, cruiseVelocity)
                .withDeadline(
                        Commands.waitUntil(
                                () -> MathUtil.isNear(position, getPosition(), TOLERANCE)))
                .withName(logKey + "/goToPosition_waitForMechanism " + position);
    }

    public Command followGoal(DoubleSupplier goal, double cruiseVelocity) {
        return run(() -> {
                    // Update cruise velocity so periodic() uses it when calling setMotionMagic
                    this.cruiseVelocity = cruiseVelocity;

                    // Create a new goal state if we don't have one or the target changed
                    if (this.goal.isEmpty() || this.goal.get().position != goal.getAsDouble()) {
                        System.out.println("**************Running");
                        this.goal = Optional.of(
                                new TrapezoidProfile.State(
                                        MathUtil.clamp(goal.getAsDouble(), EXTENDED_POS, ZERO_POS),
                                        0));
                    }
                })
                .handleInterrupt(() -> this.goal = Optional.of(setpoint))
                .beforeStarting(refreshCurrentState());
    }

    // Refreshes the current state by creating a new setpoint only if there isn't a current one
    private Command refreshCurrentState() {
        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), getVelocity()))
                .onlyIf(() -> setpoint == null || goal.isEmpty());
    }

    // Gets the current position of the wrist from the absolute encoder
    public double getPosition() {
        return WristIO.WristInputs.absoluteEncoderPosition;
    }

    @AutoLogOutput(key = "wrist/idealPosition")
    public double getIdealPosition() {
        if (goal.isPresent()) {
            return setpoint.position;
        }
        return getPosition();
    }

    // Gets the current velocity in radians per second
    public double getVelocity() {
        return WristIO.WristInputs.velocityRadiansPerSecond;
    }
}