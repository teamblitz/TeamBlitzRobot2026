package frc.robot.subsystems.funnal;

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

public class FunnalIOKraken implements FunnalIO {
    public final TalonFX funnal;

    public FunnalIOKraken() {
        funnal = new TalonFX(CAN_ID); // TODO SET VALUE

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake))
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);

        funnal.getConfigurator().apply(config);

        ControlRequest _threadInterruptStop = new NeutralOut().withUpdateFreqHz(0);
    }

    @Override
    public void setSpeed(double speed) {
        funnal.set(speed);
    }
}

