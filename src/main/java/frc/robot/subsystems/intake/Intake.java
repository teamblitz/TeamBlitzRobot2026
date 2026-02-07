package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.BlitzSubsystem;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

public class Intake extends BlitzSubsystem {
    private final IntakeIO io;

    public Intake(IntakeIO io) {
        super("intake");

        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

    }

    public Command reverse() {
        return setSpeed(REVERSE_SPEED);
    }

    public Command kick_algae() {
        return setSpeed(ALGAE_REMOVAL);
    }

    public Command shoot_coral() {
        return setSpeed(SHOOT_CORAL);
    }

    public Command coral_l1() {
        return setSpeed(L1);
    }

    private Command stop() {
        return runOnce(() -> io.setSpeed(0));
    }

    public Command setSpeed(double speed) {
        return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
    }

}
