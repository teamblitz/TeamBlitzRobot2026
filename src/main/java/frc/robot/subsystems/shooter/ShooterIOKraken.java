package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.monitor.HardwareWatchdog;
import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.Shooter.*;

public class ShooterIOKraken implements ShooterIO {
    
    public final TalonFX shooter;
    public final TalonFX feeder;

    public ShooterIOKraken() {

        shooter = new TalonFX(30);
        feeder = new TalonFX(31);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Basic configuration: neutral mode and inversion. Adjust as-needed in Constants.
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        shooter.getConfigurator().apply(config);
        feeder.getConfigurator().apply(config);

    // Register devices with watchdog
        HardwareWatchdog.getInstance().registerCTREDevice(shooter, this.getClass());
        HardwareWatchdog.getInstance().registerCTREDevice(feeder, this.getClass());

        // feeder.setControl(new Follower(shooter.getDeviceID(), MotorAlignmentValue.Aligned)); // May need to be Inversted
        config.Feedback.withSensorToMechanismRatio(SHOOTER_GEAR_RATIO);
        config.Feedback.withSensorToMechanismRatio(FEEDER_GEAR_RATIO);


    }

    @Override
    public void setShooterSpeed(double speed) {
        shooter.set(speed);
    }

    @Override
    public void setFeederSpeed(double speed) {
        feeder.set(speed);
    }

    @Override
    public void updateInputs() {
        // TODO: read real sensor values from TalonFX when available. For now, provide placeholders.
  
        Logger.recordOutput("shooter/shooterCANID", shooter.getDeviceID());
        Logger.recordOutput("shooter/feederCANID", feeder.getDeviceID());
    }
}
