package frc.robot.subsystems.wrist;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.WristConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.ControlRequest;
// import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
// import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.AsynchronousInterrupt;
// import edu.wpi.first.wpilibj.DigitalInput;

// import frc.lib.monitor.HardwareWatchdog;

import org.littletonrobotics.junction.Logger;

// import java.util.concurrent.atomic.AtomicBoolean;

public class WristIOKraken implements WristIO {
    public final TalonFX wrist;
    public final CANcoder absoluteEncoder;


    public WristIOKraken() {
        wrist = new TalonFX(1); 
        absoluteEncoder = new CANcoder(ABS_ENCODER_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT_WRIST);

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake))
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);

        wrist.getConfigurator().apply(config);
    }
    @Override
    public void updateInputs(WristInputs inputs) {
        WristInputs.velocityRadiansPerSecond = 2 * Math.PI * wrist.getVelocity().getValueAsDouble();
        //Gets the position of the absolute encoder
        WristInputs.absoluteEncoderPosition = getAbsPosition();

        Logger.recordOutput(
                "elevator/motionMagicEnabled", wrist.getMotionMagicIsRunning().getValue());
    }

    @Override
    public void setSpeed(double speed) {
        wrist.set(speed);
    }

    public double getAbsPosition() {
        var encoderPos = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        return MathUtil.angleModulus(
                (2 * Math.PI) * encoderPos);
    }
}

