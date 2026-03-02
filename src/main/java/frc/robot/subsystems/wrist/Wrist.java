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
import org.apache.commons.math3.exception.ZeroException;
import org.littletonrobotics.junction.AutoLogOutput;


public class Wrist extends BlitzSubsystem {
    private final WristIO io;

    public Wrist(WristIO io) {
        super("Wrist");

        this.io = io;

        setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
        goal = Optional.empty();

        setDefaultCommand((goToIdle()));
    }

    private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL);

    private Optional<TrapezoidProfile.State> goal;
    private TrapezoidProfile.State setpoint;

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private double cruiseVelocity = MAX_VELOCITY;

    @Override
    public void periodic() {
        super.periodic();
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        if (goal.isPresent() && DriverStation.isEnabled()) {
            System.out.println("Sending Motion Magic to: " + goal.get().position);
            io.setMotionMagic(goal.get().position);
        }

        if (DriverStation.isDisabled()) {
            goal = Optional.empty();
            io.stop();
        }
    }

    public Command upTest() {
        return startEnd(() -> io.setSpeed(0.5), () -> io.setSpeed(0));
    }

   /* public Command move_up() {
        // return startEnd(() -> io.setMotionMagic(1.64), () -> io.setSpeed(0));
        return Commands.runOnce(() -> {
            io.setMotionMagic(1);
            System.out.println("Finishes move");
        });
    }

    public Command move_down() {
        // return startEnd(() -> io.setSpeed(-0.3), () -> io.setSpeed(0));
        return Commands.runOnce(() -> {
            io.setMotionMagic(0);
        });

    } */ //Do not use

    public Command setSpeed(double speed) {
        return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
    }

    public Command goToIdle() {
        return goToPosition(ZERO_POS);
    }

    public Command goToDown() {
        return followGoal(EXTENDED_POS);
    }

    public Command goToPosition(double position) {
        return followGoal(position)
                .withDeadline(
                        Commands.waitUntil(() -> {
                            boolean near = MathUtil.isNear(position, getPosition(), TOLERANCE);
                            System.out.println("position: " + getPosition() + " target: " + position + " near: " + near);
                            return near;
                        }))
                .withName(logKey + "/goToPosition_waitForMechanism " + position);
    }

    public Command followGoal(double goal) {
        return run(() -> {
            if (this.goal.isEmpty() || this.goal.get().position != goal) {
                System.out.println("**************Setting goal to " + goal);
                this.goal = Optional.of(
                        new TrapezoidProfile.State(
                                MathUtil.clamp(goal, Math.min(EXTENDED_POS, ZERO_POS), Math.max(EXTENDED_POS, ZERO_POS)), 0));
            }
        })
                .handleInterrupt(() -> {
                    System.out.println("Interrupted, goal was: " + this.goal.map(s -> String.valueOf(s.position)).orElse("empty"));
                    this.goal = Optional.empty();
                })
                .beforeStarting(() -> System.out.println("followGoal starting"))
                .beforeStarting(refreshCurrentState());
    }

    private Command refreshCurrentState() {
        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), getVelocity()))
                .onlyIf(() -> setpoint == null || goal.isEmpty());
    }

    public double getPosition() {
        return inputs.absoluteEncoderPosition;
    }

    @AutoLogOutput(key = "wrist/idealPosition")
    public double getIdealPosition() {
//        if (goal.isPresent()) {
//            return setpoint.position;
//        }
        return getPosition();
    }

    public double getVelocity() {
        return inputs.velocityRadiansPerSecond;
    }
}