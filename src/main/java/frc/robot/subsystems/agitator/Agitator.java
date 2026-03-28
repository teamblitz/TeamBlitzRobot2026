package frc.robot.subsystems.agitator;

import static frc.robot.Constants.Agitator.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

public class Agitator extends BlitzSubsystem {
    private final AgitatorIO io;

    public Agitator(AgitatorIO io) {
        super("agitator");

        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();


    }

    public Command forward() {
        return runOnce(() ->
                                io.setSpeed(1))
                        .andThen(Commands.idle())
                        .finallyDo(() -> {
                            io.setSpeed(0);
                        });    }

    public Command reverse() {
        return runEnd(() -> io.setSpeed(-0.1), () -> io.setSpeed(0));
    }

    private Command stop() {
        return runOnce(() -> io.setSpeed(0));
    }


}
