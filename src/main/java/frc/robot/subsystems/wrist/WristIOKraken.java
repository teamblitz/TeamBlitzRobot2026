package frc.robot.subsystems.wrist;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.WristConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class WristIOKraken implements WristIO {

    public final TalonFX wrist;
    public final CANcoder absoluteEncoder;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    public WristIOKraken() {
        wrist = new TalonFX(30);
        absoluteEncoder = new CANcoder(ABS_ENCODER_ID);

        // --- CANcoder config ---
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // gives a -0.5 to 0.5 range
        // Set this to the negated raw reading when your wrist is at the zero position
        encoderConfig.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        absoluteEncoder.getConfigurator().apply(encoderConfig);

        // --- TalonFX config ---
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current limits
        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT_WRIST);

        // Motor output
        config.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);

        // FusedCANcoder — fuses absolute encoder with motor's relative encoder
        config.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO; // e.g. 45.0 for 45:1 reduction
        // config.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;         // usually 1.0 if encoder is on output shaft

        // PID gains for Motion Magic (slot 0)
        config.Slot0.kP = KP;   // tune — start around 10-40
        config.Slot0.kI = KI;   // usually 0
        config.Slot0.kD = KD;   // tune — start around 0.1-1.0
        config.Slot0.kG = KG;   // gravity feedforward — helps wrist not sag
        config.Slot0.kV = KV;   // velocity feedforward — usually ~0.12 for a Kraken

        // Motion Magic limits — these are the maximums, actual cruise velocity is set per-request
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;   // rotations per second
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCEL;         // rotations per second²
        config.MotionMagic.MotionMagicJerk = 0;                         // 0 = disabled

        wrist.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        WristInputs.velocityRadiansPerSecond = 2 * Math.PI * wrist.getVelocity().getValueAsDouble();
        WristInputs.absoluteEncoderPosition = getAbsPosition();
        WristInputs.current = wrist.getStatorCurrent().getValueAsDouble();

        Logger.recordOutput("wrist/motionMagicEnabled", wrist.getMotionMagicIsRunning().getValue());
    }

    @Override
    public void setSpeed(double speed) {
        wrist.set(speed);
    }

    @Override
    public void setMotionMagic(double position) {
        // Update cruise velocity dynamically via the configurator
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
        var encoderPos = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        return MathUtil.angleModulus((2 * Math.PI) * encoderPos);
    }
}