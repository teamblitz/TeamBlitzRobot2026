package frc.robot.subsystems.agitator;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.agitator.AgitatorIO;

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

    private Command stop() {
        return runOnce(() -> io.setSpeed(0));
    }

    public Command feed() {
        // return startEnd(() -> io.setSpeed(-0.3), () -> io.setSpeed(0));

        return runOnce(() -> io.setSpeed(-1))
                .andThen(Commands.waitSeconds(30))
                .finallyDo (
                    () -> {
                        io.setSpeed(0);
                    }
                );       
    }

    public Command reverse() {
        return startEnd(() -> io.setSpeed(1), () -> io.setSpeed(0));
    }
    public Command setSpeed(double speed) {
        return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0)); //xxx
    }
}

