package frc.robot.subsystems.spindexer;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.BlitzSubsystem;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

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

    private Command stop() {
        return runOnce(() -> io.setSpeed(0));
    }

    public Command feed() {
        return startEnd(() -> io.setSpeed(0.3), () -> io.setSpeed(0));
    }

    public Command shoot() {
        return startEnd(() -> io.setSpeed(-0.3), () -> io.setSpeed(0));
    }
    public Command setSpeed(double speed) {
        return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
    }
}

