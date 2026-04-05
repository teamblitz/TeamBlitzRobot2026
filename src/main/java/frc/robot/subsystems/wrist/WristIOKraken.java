package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import org.littletonrobotics.junction.Logger;

public class WristIOKraken implements WristIO {

  public final TalonFX wristLeft;
  public final TalonFX wristRight;
  public final CANcoder absoluteEncoderLeft;
  public final CANcoder absoluteEncoderRight;

  // Separate Motion Magic requests so we can apply independent feed forward corrections
  private final MotionMagicVoltage motionMagicLeft =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagicRight =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  public WristIOKraken() {
    wristLeft = new TalonFX(MOTOR_ID_LEFT);
    wristRight = new TalonFX(MOTOR_ID_RIGHT);
    absoluteEncoderLeft = new CANcoder(ABS_ENCODER_ID_LEFT);
    absoluteEncoderRight = new CANcoder(ABS_ENCODER_ID_RIGHT);

    absoluteEncoderLeft
        .getConfigurator()
        .apply(buildEncoderConfig(MAGNET_OFFSET_LEFT, ENCODER_INVERTED_LEFT));
    absoluteEncoderRight
        .getConfigurator()
        .apply(buildEncoderConfig(MAGNET_OFFSET_RIGHT, ENCODER_INVERTED_RIGHT));

    wristLeft.getConfigurator().apply(buildMotorConfig(absoluteEncoderLeft, MOTOR_INVERTED_LEFT));
    wristRight
        .getConfigurator()
        .apply(buildMotorConfig(absoluteEncoderRight, MOTOR_INVERTED_RIGHT));
  }

  private CANcoderConfiguration buildEncoderConfig(double magnetOffset, boolean inverted) {
    CANcoderConfiguration cfg = new CANcoderConfiguration();
    cfg.MagnetSensor.SensorDirection =
        inverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;
    cfg.MagnetSensor.MagnetOffset = magnetOffset;
    return cfg;
  }

  private TalonFXConfiguration buildMotorConfig(CANcoder encoder, boolean inverted) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_WRIST;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    cfg.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
        .withInverted(
            inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

    // Each motor uses its own encoder as the feedback source
    cfg.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;

    cfg.Slot0.kP = KP;
    cfg.Slot0.kI = KI;
    cfg.Slot0.kD = KD;
    cfg.Slot0.kG = KG;
    cfg.Slot0.kV = KV;
    cfg.Slot0.kS = KS;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    cfg.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
    cfg.MotionMagic.MotionMagicAcceleration = MAX_ACCEL;
    cfg.MotionMagic.MotionMagicJerk = 0;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_LIMIT_FORWARD;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_LIMIT_REVERSE;

    return cfg;
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    double leftPos = absoluteEncoderLeft.getAbsolutePosition().getValueAsDouble();
    double rightPos = absoluteEncoderRight.getAbsolutePosition().getValueAsDouble();

    inputs.absoluteEncoderPositionLeft = leftPos;
    inputs.velocityRadiansPerSecondLeft = wristLeft.getVelocity().getValueAsDouble();
    inputs.currentLeft = wristLeft.getStatorCurrent().getValueAsDouble();

    inputs.absoluteEncoderPositionRight = rightPos;
    inputs.velocityRadiansPerSecondRight = wristRight.getVelocity().getValueAsDouble();
    inputs.currentRight = wristRight.getStatorCurrent().getValueAsDouble();

    inputs.absoluteEncoderPosition = (leftPos + rightPos) / 2.0;
    inputs.encoderDelta = Math.abs(leftPos - rightPos);

    Logger.recordOutput("wrist/leftMotorPosition", wristLeft.getPosition().getValueAsDouble());
    Logger.recordOutput("wrist/rightMotorPosition", wristRight.getPosition().getValueAsDouble());
    Logger.recordOutput("wrist/leftAbsPos", leftPos);
    Logger.recordOutput("wrist/rightAbsPos", rightPos);
    Logger.recordOutput("wrist/encoderDelta", inputs.encoderDelta);
    Logger.recordOutput("wrist/avgPosition", inputs.absoluteEncoderPosition);
  }

  @Override
  public void setSpeed(double speed) {
    wristLeft.set(speed);
    wristRight.set(speed);
  }

  @Override
  public void setMotionMagic(double position) {
    double leftPos = absoluteEncoderLeft.getAbsolutePosition().getValueAsDouble();
    double rightPos = absoluteEncoderRight.getAbsolutePosition().getValueAsDouble();

    // Positive delta means left is ahead of right
    // We apply a small feed forward nudge to slow down whichever side is ahead
    double delta = leftPos - rightPos;
    double correction = delta * SYNC_CORRECTION_SCALE;

    // Left is ahead  -> correction is positive -> subtract from left, add to right
    // Left is behind -> correction is negative -> add to left, subtract from right
    wristLeft.setControl(motionMagicLeft.withPosition(position).withFeedForward(-correction));
    wristRight.setControl(motionMagicRight.withPosition(position).withFeedForward(correction));

    Logger.recordOutput("wrist/syncDelta", delta);
    Logger.recordOutput("wrist/syncCorrection", correction);
  }

  @Override
  public void stop() {
    wristLeft.setControl(neutralRequest);
    wristRight.setControl(neutralRequest);
  }
}
