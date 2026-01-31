package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import frc.robot.subsystems.intake.IntakeIO;

public class Wrist extends BlitzSubsystem {
    private final WristIO io;

    public Wrist(WristIO io) {
        super("Wrist");

        this.io = io;
    }

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
}
