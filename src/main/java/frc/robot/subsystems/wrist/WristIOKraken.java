package frc.robot.subsystems.wrist;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.Constants.WristConstants.INVERTED;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import org.littletonrobotics.junction.Logger;

public class WristIOKraken implements WristIO {

  /*   Motors   */
  public final TalonFX rightWrist;
  public final TalonFX leftWrist;

  /*   Absolute Encoders   */
  public final CANcoder rightAbsoluteEncoder;
  public final CANcoder leftAbsoluteEncoder;

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final Follower followerRequest;

  public WristIOKraken() {
    rightWrist = new TalonFX(18); // TODO set
    leftWrist = new TalonFX(20); // TODO set
    rightAbsoluteEncoder = new CANcoder(19); // TODO set
    leftAbsoluteEncoder = new CANcoder(21); // TODO set

    /*   Motor Synchronization   */
    followerRequest =
        new Follower(
            leftWrist.getDeviceID(), MotorAlignmentValue.Aligned); // May need to be opposed

    /*   Right Absolute Encoder Config   */
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive; // May need to be Clockwise_Positive
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // TODO retune
    encoderConfig.MagnetSensor.MagnetOffset = -0.8; // TODO retune
    rightAbsoluteEncoder.getConfigurator().apply(encoderConfig);

    /*   Left Absolute Encoder Config   */
    CANcoderConfiguration encoderConfig2 = new CANcoderConfiguration();
    encoderConfig2.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig2.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoderConfig2.MagnetSensor.MagnetOffset = -0.154; // TODO retune
    leftAbsoluteEncoder.getConfigurator().apply(encoderConfig2);

    /*   Right Side Motor - Primary Motor   */
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_WRIST;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(
            INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
    config.Feedback.FeedbackRemoteSensorID = rightAbsoluteEncoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;
    config.Slot0.kP = KP;
    config.Slot0.kI = KI;
    config.Slot0.kD = KD;
    config.Slot0.kG = KG;
    config.Slot0.kV = KV;
    config.Slot0.kS = KS;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MAX_ACCEL;
    config.MotionMagic.MotionMagicJerk = 0;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_LIMIT_FORWARD;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_LIMIT_REVERSE;
    rightWrist.getConfigurator().apply(config);

    /*   Left Side Motor - Follower, may not need gains   */
    TalonFXConfiguration config2 = new TalonFXConfiguration();
    config2.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_WRIST;
    config2.CurrentLimits.StatorCurrentLimitEnable = true;
    config2.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(
            INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
    leftWrist.getConfigurator().apply(config2);
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    //   Applying the follower control again to ensure that the left motor follower is correctly
    // applied
    leftWrist.setControl(followerRequest);

    boolean primaryOk = rightAbsoluteEncoder.getAbsolutePosition().getStatus().isOK();
    double position =
        primaryOk
            ? rightAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()
            : leftAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();

    inputs.velocityRadiansPerSecond = rightWrist.getVelocity().getValueAsDouble();
    inputs.absoluteEncoderPosition = position;
    inputs.current = rightWrist.getStatorCurrent().getValueAsDouble();

    Logger.recordOutput("wrist/motorPosition", rightWrist.getPosition().getValueAsDouble());
    Logger.recordOutput("wrist/absoluteEncoderPosition", position);
    Logger.recordOutput(
        "wrist/encoder1Position", rightAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput(
        "wrist/encoder2Position", leftAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput("wrist/usingPrimaryEncoder", primaryOk);
    Logger.recordOutput(
        "wrist/motionMagicRunning", rightWrist.getMotionMagicIsRunning().getValue());
    Logger.recordOutput("wrist/motor1Current", rightWrist.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("wrist/motor2Current", leftWrist.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void setSpeed(double speed) {
    rightWrist.set(speed);
  }

  @Override
  public void setMotionMagic(double position) {
    rightWrist.setControl(motionMagicRequest.withPosition(position));
  }

  @Override
  public void stop() {
    rightWrist.setControl(neutralRequest);
  }

  public double getAbsPosition() {
    boolean primaryOk = rightAbsoluteEncoder.getAbsolutePosition().getStatus().isOK();
    return primaryOk
        ? rightAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()
        : leftAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
  }
}
