package frc.robot.subsystems.spindexer;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.spindexer.SpindexerIO;

public class Spindexer extends BlitzSubsystem {
    private final SpindexerIO io;

    public Spindexer(SpindexerIO io) {
        super("spindexer");
        
      this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Command stop() {
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

