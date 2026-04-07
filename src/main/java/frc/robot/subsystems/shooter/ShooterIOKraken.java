package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants.*;
import org.littletonrobotics.junction.Logger;

public class ShooterIOKraken implements ShooterIO {

  public final TalonFX leftShooter;
  public final TalonFX rightShooter;
  public final TalonFX feeder;

  public ShooterIOKraken() {
    leftShooter = new TalonFX(RIGHT_SHOOTER_ID);
    rightShooter = new TalonFX(LEFT_SHOOTER_ID);
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

    rightShooter.setControl(new Follower(leftShooter.getDeviceID(), MotorAlignmentValue.Aligned));

    leftShooter.getConfigurator().apply(shooterConfig);
    rightShooter.getConfigurator().apply(feederConfig);
    feeder.getConfigurator().apply(feederConfig);
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
