package frc.robot.subsystems.wrist;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.Constants.WristConstants.INVERTED;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class WristIOKraken implements WristIO {

    public final TalonFX wrist;
    public final CANcoder absoluteEncoder;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    public WristIOKraken() {
        wrist = new TalonFX(21);
        absoluteEncoder = new CANcoder(ABS_ENCODER_ID);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.73;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;
        // Set this to the negated raw reading when wrist is at the zero position
        encoderConfig.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        absoluteEncoder.getConfigurator().apply(encoderConfig);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_WRIST;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);

        config.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;  //Fused if we want pro
        // config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
        config.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;

        // PID gains for Motion Magic (slot 0)
        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;
        config.Slot0.kG = KG;
        config.Slot0.kV = KV;
        config.Slot0.kS = KS;

        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCEL;
        config.MotionMagic.MotionMagicJerk = 0;

         config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // config.Slot0.GravityArmPositionOffset = IDLE_POS;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_LIMIT_FORWARD; // small buffer past zero
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_LIMIT_REVERSE; // small buffer before zero

        wrist.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        inputs.velocityRadiansPerSecond = wrist.getVelocity().getValueAsDouble();
        inputs.absoluteEncoderPosition = getAbsPosition();
        inputs.current = wrist.getStatorCurrent().getValueAsDouble();

        Logger.recordOutput("wrist/motorPosition", wrist.getPosition().getValueAsDouble());
        Logger.recordOutput("wrist/absoluteEncoderPosition", getAbsPosition());
        Logger.recordOutput("wrist/motionMagicEnabled", wrist.getMotionMagicIsRunning().getValue());    }

    @Override
    public void setSpeed(double speed) {
        wrist.set(speed);
    }

    @Override
    public void setMotionMagic(double position) {
        // var mmConfig = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        // mmConfig.MotionMagicCruiseVelocity = MAX_VELOCITY;
        // mmConfig.MotionMagicAcceleration = MAX_ACCEL;
        // mmConfig.MotionMagicJerk = 0;
        // wrist.getConfigurator().apply(mmConfig);

        // wrist.setControl(motionMagicRequest.withPosition(position / 2 * Math.PI));
        // wrist.setPosition(position, 2);
        wrist.setControl(motionMagicRequest.withPosition(position));
    }

    @Override
    public void stop() {
        wrist.setControl(neutralRequest);
    }

    public double getAbsPosition() {
//        var encoderPos = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }
}
