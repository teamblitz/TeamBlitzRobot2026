package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.BlitzSubsystem;

public class Shooter extends BlitzSubsystem {
	private final ShooterIO io;

	public Shooter(ShooterIO io) {
		super("shooter");

		this.io = io;
	}

	@Override
	public void periodic() {
		super.periodic();
		// IO update would be done by a higher-level manager; keep minimal here
	}

	public Command setShooterSpeed(double speed) {
		return startEnd(() -> io.setShooterSpeed(speed), () -> io.setShooterSpeed(0));
	}

	public Command setFeederSpeed(double speed) {
		return startEnd(() -> io.setFeederSpeed(speed), () -> io.setFeederSpeed(0));
	}
}
