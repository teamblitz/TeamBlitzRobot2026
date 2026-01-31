package frc.robot.subsystems.wrist;

import static frc.robot.Constants.Intake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.lib.monitor.HardwareWatchdog;

import org.littletonrobotics.junction.Logger;

import java.util.concurrent.atomic.AtomicBoolean;

public class WristIOKraken implements WristIO {
    public final TalonFX wrist;

    public WristIOKraken() {
        wrist = new TalonFX(0); // TODO SET VALUE

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake))
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);

        wrist.getConfigurator().apply(config);
    }

    @Override
    public void setSpeed(double speed) {
        wrist.set(speed);
    }
}

