package frc.robot.subsystems.wrist;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;

import static frc.robot.Constants.WristConstants.*;


public class Wrist extends BlitzSubsystem {
    private final WristIO io;
    private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();
    private Optional<Double> goal = Optional.empty();

    public Wrist(WristIO io) {
        super("Wrist");
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        // if (goal.isPresent() && DriverStation.isEnabled()) {
        //     System.out.println("Sending Motion Magic to: " + goal.get());
        //     io.setMotionMagic(goal.get());
        // }

        if (DriverStation.isDisabled()) {
            goal = Optional.empty();
            io.stop();
        }

    }

    public Command goToIdle() {
        return goToPosition(IDLE_POS);
    }

    public Command goToDown() {
        return goToPosition(EXTENDED_POS);
    }

    public Command goToPosition(Double position) {
        return Commands.runOnce(() -> {
            //this is not really needed but here for example
            boolean near = MathUtil.isNear(position, getPosition(), TOLERANCE);
            if(near) return; //exit if its already there

            //sets the goal to keep it around in case we want to log or use it in periodic.
            if (this.goal.isEmpty() || this.goal.get() != position) {
                this.goal = Optional.of(position);
            }

            io.setMotionMagic(position);
            System.out.println("Finished goToPosition with position:" + position);
        });
    }

    public double getPosition() {
        return inputs.absoluteEncoderPosition;
    }

    public double getVelocity() {
        return inputs.velocityRadiansPerSecond;
    }

    // public Command goToPositionOld(double position) {
    //     return followGoal(position)
    //             .withDeadline(
    //                     Commands.waitUntil(() -> {
    //                         boolean near = MathUtil.isNear(position, getPosition(), TOLERANCE);
    //                         System.out.println("position: " + getPosition() + " target: " + position + " near: " + near);
    //                         return near;
    //                     }))
    //             .withName(logKey + "/goToPosition_waitForMechanism " + position);
    // }

    // public Command followGoal(double goal) {
    //     return run(() -> {
    //         if (this.goal.isEmpty() || this.goal.get().position != goal) {
    //             System.out.println("**************Setting goal to " + goal);
    //             this.goal = Optional.of(new TrapezoidProfile.State(
    //                     MathUtil.clamp(goal, Math.min(EXTENDED_POS, IDLE_POS), Math.max(EXTENDED_POS, IDLE_POS)), 0));
    //         }
    //     })
    //             .handleInterrupt(() -> {
    //                 System.out.println("Interrupted, goal was: " + this.goal.map(s -> String.valueOf(s.position)).orElse("empty"));
    //                 this.goal = Optional.empty();
    //             })
    //             .beforeStarting(() -> System.out.println("followGoal starting"));
    // }

//    private Command refreshCurrentState() {
//        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), getVelocity()))
//                .onlyIf(() -> setpoint == null || goal.isEmpty());
//    }

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

    // public Command upTest() {
    //     return startEnd(() -> io.setSpeed(0.5), () -> io.setSpeed(0));
    // }

//     @AutoLogOutput(key = "wrist/idealPosition")
//     public double getIdealPosition() {
// //        if (goal.isPresent()) {
// //            return setpoint.position;
// //        }
//         return getPosition();
//     }


}