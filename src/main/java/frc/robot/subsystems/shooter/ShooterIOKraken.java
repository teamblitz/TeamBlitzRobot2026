package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants.*;

import frc.lib.monitor.HardwareWatchdog;

import static frc.robot.Constants.ShooterConstants.RAMP;

import org.littletonrobotics.junction.Logger;

public class ShooterIOKraken implements ShooterIO {
    
    public final TalonFX leftShooter;
    public final TalonFX rightShooter;
    public final TalonFX feeder;

    public ShooterIOKraken() {
        leftShooter = new TalonFX(0);
        rightShooter = new TalonFX(1);
        feeder = new TalonFX(2);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
        shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = RAMP;

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        rightShooter.setControl(new Follower(leftShooter.getDeviceID(), MotorAlignmentValue.Aligned)); // May need to be Inverted


        leftShooter.getConfigurator().apply(shooterConfig);
        rightShooter.getConfigurator().apply(feederConfig);
        feeder.getConfigurator().apply(feederConfig);

        HardwareWatchdog.getInstance().registerCTREDevice(leftShooter, this.getClass());
        HardwareWatchdog.getInstance().registerCTREDevice(rightShooter, this.getClass());
        HardwareWatchdog.getInstance().registerCTREDevice(feeder, this.getClass());
    }

     @Override
     public void setShooterSpeed(double speed) {
         leftShooter.set(speed);
     }

     @Override
     public void setFeederSpeed(double speed) {
         feeder.set(speed);
     }

     @Override
     public void updateInputs() {
         // TODO: read real sensor values from TalonFX when available. For now, provide placeholders.
  
         Logger.recordOutput("shooter/shooterCANID", leftShooter.getDeviceID());
         Logger.recordOutput("shooter/feederCANID", rightShooter.getDeviceID());
         Logger.recordOutput("shooter/deepFeedCANID", feeder.getDeviceID());
     }
}

