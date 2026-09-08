package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants.ShooterConstants.*;
import org.littletonrobotics.junction.Logger;

public class ShooterIOKraken implements ShooterIO {

  public final TalonFX rightShooter;
  public final TalonFX feeder;

  public final CANcoder absoluteEncoderShooter;

  public ShooterIOKraken() {
    rightShooter = new TalonFX(RIGHT_SHOOTER_ID);
    feeder = new TalonFX(FEEDER_ID);

    absoluteEncoderShooter = new CANcoder(ABS_ENCODER_ID_SHOOTER);

    absoluteEncoderShooter
        .getConfigurator()
        .apply(buildEncoderConfig(MAGNET_OFFSET_SHOOTER, ENCODER_INVERTED_SHOOTER), 0.1);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = RAMP;

    shooterConfig.Slot0.kS = 0.05;
    shooterConfig.Slot0.kV = 0.12;
    shooterConfig.Slot0.kP = 0.11;

    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    rightShooter.getConfigurator().apply(shooterConfig);
    feeder.getConfigurator().apply(feederConfig);
  }

  private CANcoderConfiguration buildEncoderConfig(double magnetOffset, boolean inverted) {
    CANcoderConfiguration cfg = new CANcoderConfiguration();
    cfg.MagnetSensor.SensorDirection =
        inverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    cfg.MagnetSensor.MagnetOffset = magnetOffset;
    return cfg;
  }

  @Override
  public void setShooterSpeed(double speed) {
    rightShooter.set(speed);
  }

  @Override
  public void setFeederSpeed(double speed) {
    feeder.set(speed);
  }

  public void setShooterVoltage(VelocityVoltage voltage) {
    rightShooter.setControl(voltage);
  }

  @Override
  public double getShooterRPS() {
    return rightShooter.getVelocity().getValueAsDouble();
  }

  public double getEncoderRPM() {
    return 0;
  }

  @Override
  public void updateInputs() {
    Logger.recordOutput("shooter/shooterCANID", rightShooter.getDeviceID());
    Logger.recordOutput("shooter/feederCANID", feeder.getDeviceID());
  }
}
