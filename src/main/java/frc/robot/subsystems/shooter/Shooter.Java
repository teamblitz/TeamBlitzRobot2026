package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.BlitzSubsystem;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	private final ShooterIO io;

    private final TalonFX shooter;
    private final TalonFX feeder;

	public Shooter() {
	
        shooter = new TalonFX(30);
        feeder = new TalonFX(31);
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
                .finallyDo(
                    () -> {
                        shooter.set(0);
                        feeder.set(0);
                    }
                );

    }
}
