package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.monitor.HardwareWatchdog;
import org.littletonrobotics.junction.Logger;

public class ShooterIOKraken implements ShooterIO {
    
    public final TalonFX shooter;
    public final TalonFX feeder;
    public final TalonFX deepFeed;

    public ShooterIOKraken() {
        shooter = new TalonFX(30);
        feeder = new TalonFX(31);
        deepFeed = new TalonFX(32);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
        shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        feeder.setControl(new Follower(shooter.getDeviceID(), MotorAlignmentValue.Opposed)); // May need to be Inverted


        shooter.getConfigurator().apply(shooterConfig);
        feeder.getConfigurator().apply(feederConfig);
        deepFeed.getConfigurator().apply(feederConfig);

        HardwareWatchdog.getInstance().registerCTREDevice(shooter, this.getClass());
        HardwareWatchdog.getInstance().registerCTREDevice(feeder, this.getClass());
        HardwareWatchdog.getInstance().registerCTREDevice(deepFeed, this.getClass());
    }

     @Override
     public void setShooterSpeed(double speed) {
         shooter.set(speed);
     }

     @Override
     public void setFeederSpeed(double speed) {
         deepFeed.set(speed);
     }

     @Override
     public void updateInputs() {
         // TODO: read real sensor values from TalonFX when available. For now, provide placeholders.
  
         Logger.recordOutput("shooter/shooterCANID", shooter.getDeviceID());
         Logger.recordOutput("shooter/feederCANID", feeder.getDeviceID());
         Logger.recordOutput("shooter/deepFeedCANID", deepFeed.getDeviceID());
     }
}

