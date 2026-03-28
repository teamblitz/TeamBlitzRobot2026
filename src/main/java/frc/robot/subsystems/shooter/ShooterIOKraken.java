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

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterIOKraken implements ShooterIO {
    
    public final TalonFX rightShooter;
    public final TalonFX leftShooter;
    public final TalonFX feeder;

    public ShooterIOKraken() {
        rightShooter = new TalonFX(RIGHT_SHOOTER_ID);
        leftShooter = new TalonFX(LEFT_SHOOTER_ID);
        feeder = new TalonFX(FEEDER_ID);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
        shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = RAMP;

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        leftShooter.setControl(new Follower(rightShooter.getDeviceID(), MotorAlignmentValue.Aligned)); // May need to be Inverted


        rightShooter.getConfigurator().apply(shooterConfig);
        leftShooter.getConfigurator().apply(feederConfig);
        feeder.getConfigurator().apply(feederConfig);

        HardwareWatchdog.getInstance().registerCTREDevice(rightShooter, this.getClass());
        HardwareWatchdog.getInstance().registerCTREDevice(leftShooter, this.getClass());
        HardwareWatchdog.getInstance().registerCTREDevice(feeder, this.getClass());
    }

     @Override
     public void setShooterSpeed(double speed) {
         rightShooter.set(speed);
     }

     @Override
     public void setFeederSpeed(double speed) {
         feeder.set(speed);
     }

     @Override
     public void updateInputs(ShooterInputs inputs) {
         // TODO: read real sensor values from TalonFX when available. For now, provide placeholders.
         inputs.rpm = rightShooter.getVelocity().getValueAsDouble();
         inputs.current = rightShooter.getSupplyCurrent().getValueAsDouble();

         inputs.rpm = leftShooter.getVelocity().getValueAsDouble();
         inputs.current = leftShooter.getSupplyCurrent().getValueAsDouble();
  
         Logger.recordOutput("shooter/shooterCANID", rightShooter.getDeviceID());
         Logger.recordOutput("shooter/feederCANID", leftShooter.getDeviceID());
         Logger.recordOutput("shooter/deepFeedCANID", feeder.getDeviceID());
     }
}

