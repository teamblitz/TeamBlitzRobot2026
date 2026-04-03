package frc.robot.subsystems.agitator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import frc.robot.subsystems.agitator.AgitatorIOKraken;

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

    public Command run() {
        return runEnd(() -> io.setSpeed(0.6), () -> io.setSpeed(0));
    }
}