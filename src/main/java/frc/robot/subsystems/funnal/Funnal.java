package frc.robot.subsystems.funnal;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.BlitzSubsystem;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

public class Funnal extends BlitzSubsystem {
    private final Funnal io;

    public Funnal(FunnalIO io) {
        super("funnal");
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
