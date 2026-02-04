package frc.robot.subsystems.wrist;

import static frc.robot.Constants.Wrist.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;



public class Wrist extends BlitzSubsystem {
    private final WristIO io;

    public Wrist(WristIO io) {
        super("Wrist");

        this.io = io;
    }

    private Optional<TrapezoidProfile.State> goal;
    private TrapezoidProfile.State setpoint;

    @Override
    public void periodic() {
        super.periodic();
    }

    public Command move_up() {
        return startEnd(()-> io.setSpeed(0.3), () -> io.setSpeed(0));
    }

    public Command move_down() {
        return startEnd(() -> io.setSpeed(0.3), () -> io.setSpeed(0));
    }


    public Command setSpeed(double speed) {
        return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
    }

    //A method to get the current position of the wrist
    public double getPosition() {
        //Return the position of the encoder
        return WristIO.WristInputs.absoluteEncoderPosition;
    }
    //Sends the motor to a position by running followGoal until the deadline is called
    public Command goToPositon(double position) {

        return followGoal(() -> position)
                    .withDeadline(
                        //This just means wait until the robot gets close to the desired position
                            Commands.waitUntil(
                                    () -> MathUtil.isNear(position, getPosition(), TOLERANCE)))
                    .withName(logKey + "/goToPosition_waitForMechanism " + position);
    }
    public Command followGoal(DoubleSupplier goal) {
        //Run this process
        return run(() -> {
            // Create a new trapezoid profile if we don't have a goal currently set
            if (this.goal.isEmpty() || this.goal.get().position != goal.getAsDouble()) {
                this.goal =
                        Optional.of(
                            new TrapezoidProfile.State(
                                MathUtil.clamp(
                                    goal.getAsDouble(), MIN_POS, MAX_POS),
                                 0));
                    }
                })
                .handleInterrupt(() -> this.goal = Optional.of(setpoint))
                .beforeStarting(refreshCurrentState());
    }
    
    //Refreshes your current state by creating a new setpoint only if there isn't a current one
    private Command refreshCurrentState() {
        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), getVelocity()))
                .onlyIf(() -> setpoint == null || goal.isEmpty());
    }

    //Gets the current velocity in radians per second
    public double getVelocity() {
        return WristIO.WristInputs.velocityRadiansPerSecond;
    }
    //OLD CODE

    //     public Command goToPosition(double position, boolean requireProfileCompletion) {
    //     if (requireProfileCompletion)
    //         return followGoal(() -> position)
    //                 .withDeadline(
    //                         Commands.waitUntil(
    //                                 () -> MathUtil.isNear(position, getPosition(), TOLERANCE)))
    //                 .withName(logKey + "/goToPosition_waitForMechanism " + position);
    //     else
    //         return followGoal(() -> position)
    //                 .withDeadline(
    //                         Commands.waitUntil(
    //                                         () ->
    //                                                 MathUtil.isNear(
    //                                                         position, getIdealPosition(), 1e-9))
    //                                 .withName(logKey + "/goToPosition_waitForProfile " + position));
    // }
}
