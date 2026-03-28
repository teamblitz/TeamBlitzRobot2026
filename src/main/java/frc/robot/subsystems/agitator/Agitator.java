package frc.robot.subsystems.agitator;

import frc.lib.BlitzSubsystem;

public class Agitator extends BlitzSubsystem {
    private final AgitatorIO io;

    public Agitator() {
        super("intake");

        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}