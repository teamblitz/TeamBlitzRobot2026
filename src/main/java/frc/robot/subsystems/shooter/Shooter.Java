package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {
	// private final ShooterIO io;

    private final TalonFX shooter;
    private final TalonFX feeder;
    //private final TalonFX deepFeed;

	public Shooter() {
	
        shooter = new TalonFX(30);
        feeder = new TalonFX(31);
        //deepFeed = new TalonFX(32);
	}

	@Override
	public void periodic() {
		super.periodic();
		// IO update would be done by a higher-level manager; keep minimal here
	}


    public Command shoot() {
        return runOnce(() -> shooter.set(-1))
                .andThen(Commands.waitSeconds(1))
                .andThen(() -> feeder.set(1))
                .andThen(Commands.waitSeconds(100))
               // .andThen(() -> deepFeed.set(1))
               // .andThen(Commands.waitSeconds(1))//TODO set values to run
                .finallyDo(
                    () -> {
                        shooter.set(0);
                        feeder.set(0);
                    }
                );

    }
}
