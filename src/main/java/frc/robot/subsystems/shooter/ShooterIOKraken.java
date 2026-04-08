package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants.*;
import org.littletonrobotics.junction.Logger;

public class ShooterIOKraken implements ShooterIO {

  public final TalonFX rightShooter;
  public final TalonFX feeder;

  public ShooterIOKraken() {
    rightShooter = new TalonFX(RIGHT_SHOOTER_ID);
    feeder = new TalonFX(FEEDER_ID);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = RAMP;

    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    rightShooter.getConfigurator().apply(shooterConfig);
    feeder.getConfigurator().apply(feederConfig);
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
  public void updateInputs() {
    Logger.recordOutput("shooter/shooterCANID", rightShooter.getDeviceID());
    Logger.recordOutput("shooter/feederCANID", feeder.getDeviceID());
  }
}
