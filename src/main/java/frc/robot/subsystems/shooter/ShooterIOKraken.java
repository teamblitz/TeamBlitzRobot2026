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

        shooter = new TalonFX(30);//TODO Set motor val
        feeder = new TalonFX(31);//TODO Set motor val
        
        //Follower feeder = new Follower(30, false);
        deepFeed = new TalonFX(32);//TODO Set motor val

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Basic configuration: neutral mode and inversion. Adjust as-needed in Constants.
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        shooter.getConfigurator().apply(config);
        feeder.getConfigurator().apply(config);
        deepFeed.getConfigurator().apply(config);
 
        feeder.setControl(new Follower(shooter.getDeviceID(), MotorAlignmentValue.Aligned)); // May need to be Inversted
        //feeder.setControl(new Follower(30, true));
        

    // Register devices with watchdog
        HardwareWatchdog.getInstance().registerCTREDevice(shooter, this.getClass());
        HardwareWatchdog.getInstance().registerCTREDevice(feeder, this.getClass());
        HardwareWatchdog.getInstance().registerCTREDevice(deepFeed, this.getClass());

//        private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    }

    // @Override
    // public void setShooterSpeed(double speed) {
    //     shooter.set(speed);
    // }

    // @Override
    // public void setFeederSpeed(double speed) {
    //     feeder.set(speed);
    // }

    // @Override
    // public void updateInputs() {
    //     // TODO: read real sensor values from TalonFX when available. For now, provide placeholders.
  
    //     Logger.recordOutput("shooter/shooterCANID", shooter.getDeviceID());
    //     Logger.recordOutput("shooter/feederCANID", feeder.getDeviceID());
    //     Logger.recordOutput("shooter/deepFeedCANID", deepFeed.getDeviceID());  
    // }
}

