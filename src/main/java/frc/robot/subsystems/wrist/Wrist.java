package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends BlitzSubsystem {
    public final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL);
    private final TrapezoidProfile profile;

    private Optional<TrapezoidProfile.State> goal = Optional.empty();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public Wrist(WristIO io) {
        super("Wrist");
        this.io = io;
        this.profile = new TrapezoidProfile(constraints);
    }

    @Override
    public void periodic() {
        super.periodic();
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        goal.ifPresent(g -> {
            setpoint = profile.calculate(0.02, setpoint, g);
            io.setPosition(setpoint.position);
        });
    }

    public double getPosition() {
        return inputs.absoluteEncoderPosition;
    }

    public double getVelocity() {
        return inputs.velocityRadiansPerSecond;
    }

    public Command moveUp() {
        return startEnd(() -> io.setSpeed(0.3), () -> io.setSpeed(0));
    }

    public Command moveDown() {
        return startEnd(() -> io.setSpeed(-0.3), () -> io.setSpeed(0));
    }

    public Command setSpeed(double speed) {
        return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
    }

    public Command goToPosition(double position) {
        return followGoal(() -> position)
                .withDeadline(
                        Commands.waitUntil(
                                () -> MathUtil.isNear(position, getPosition(), TOLERANCE)))
                .withName(logKey + "/goToPosition " + position);
    }

    public Command followGoal(DoubleSupplier goal) {
        return run(() -> {
                    if (this.goal.isEmpty() || this.goal.get().position != goal.getAsDouble()) {
                        this.goal = Optional.of(
                                new TrapezoidProfile.State(
                                        MathUtil.clamp(goal.getAsDouble(), EXTENDED_POS, ZERO_POS),
                                        0));
                    }
                })
                .handleInterrupt(() -> this.goal = Optional.of(setpoint))
                .beforeStarting(refreshCurrentState());
    }

    private Command refreshCurrentState() {
        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), getVelocity()))
                .onlyIf(() -> setpoint == null || goal.isEmpty());
    }
}