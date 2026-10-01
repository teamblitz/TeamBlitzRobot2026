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

  /*
   * Creates new shooters called:
   * rightshooter,
   * feeder,
   * and sets ID values to these moters.
   */
  public ShooterIOKraken() {
    rightShooter = new TalonFX(RIGHT_SHOOTER_ID);
    feeder = new TalonFX(FEEDER_ID);

    // Creates a new Cancoder Object
    absoluteEncoderShooter = new CANcoder(ABS_ENCODER_ID_SHOOTER);

    // Configs the new CANcoder
    absoluteEncoderShooter
        .getConfigurator()
        .apply(buildEncoderConfig(MAGNET_OFFSET_SHOOTER, ENCODER_INVERTED_SHOOTER), 0.1);

    // Configs the rightShooter moter.
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = RAMP;

    shooterConfig.Slot0.kS = 0.05;
    shooterConfig.Slot0.kV = 0.12;
    shooterConfig.Slot0.kP = 0.11;

    // Configs the feeder moter.
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    rightShooter.getConfigurator().apply(shooterConfig);
    feeder.getConfigurator().apply(feederConfig);
  }

  /*
   * Creates a new CANcoderConfigutation.
   * Sets the magnetOffset, and if it is inverted or not.
   *
   * @return the CANcoder configuration (cfg)
   */
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

  // Creates a void that sets the rightShooter speed when called, using the double speed.
  @Override
  public void setShooterSpeed(double speed) {
    rightShooter.set(speed);
  }

  // Creates a void that sets the feeder speed when called, using the double speed.
  @Override
  public void setFeederSpeed(double speed) {
    feeder.set(speed);
  }

  // Creates a void that allows the changing of voltage for the rightShooter moter.
  public void setShooterVoltage(VelocityVoltage voltage) {
    rightShooter.setControl(voltage);
  }

  // Creates a double that gets the RPS of the rightShooter moter.
  @Override
  public double getShooterRPS() {
    return rightShooter.getVelocity().getValueAsDouble();
  }

  // Creates a double that gets the RPM of the RightShooter moter.
  public double getEncoderRPM() {
    return 0;
  }

  /*
   * Using the UpdateInputs, we log multiple things.
   * The rightShooter ID
   * The Feeder ID
   */
  @Override
  public void updateInputs() {
    Logger.recordOutput("shooter/shooterCANID", rightShooter.getDeviceID());
    Logger.recordOutput("shooter/feederCANID", feeder.getDeviceID());
  }
}
