package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class WristIOKraken implements WristIO {
    private final TalonFX wrist;
    private final CANcoder absoluteEncoder;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public WristIOKraken() {
        wrist = new TalonFX(WRIST_ID);
        absoluteEncoder = new CANcoder(ABS_ENCODER_ID);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(encoderConfig);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.FeedbackRemoteSensorID = ABS_ENCODER_ID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = WRIST_GEAR_RATIO;
        config.Feedback.SensorToMechanismRatio = 1.0;

        config.Slot0.kP = 0.0; // tune
        config.Slot0.kD = 0.0; // tune
        config.Slot0.kG = 0.0; // tune
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = 0.0; // tune
        config.MotionMagic.MotionMagicAcceleration = 0.0;   // tune

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT_WRIST);
        config.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive);

        wrist.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        inputs.velocityRadiansPerSecond = 2 * Math.PI * wrist.getVelocity().getValueAsDouble();
        inputs.absoluteEncoderPosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        inputs.currentAmps = wrist.getSupplyCurrent().getValueAsDouble();
        inputs.rpm = wrist.getVelocity().getValueAsDouble() * 60.0;
        Logger.recordOutput("wrist/motionMagicEnabled", wrist.getMotionMagicIsRunning().getValue());
    }

    @Override
    public void setSpeed(double speed) {
        wrist.set(speed);
    }

    @Override
    public void setPosition(double positionRotations) {
        wrist.setControl(motionMagicRequest.withPosition(positionRotations));
    }
}