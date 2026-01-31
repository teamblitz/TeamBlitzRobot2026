package frc.robot.subsystems;

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




    public Command setSpeed(double speed) {
        return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
    }
}
