package frc.robot.subsystems.spindexer;

import static frc.robot.Constants.Spindexer.*;
import static frc.robot.Constants.Intake.*;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SpindexerIOKraken implements SpindexerIO {
    public final TalonFX spindexer;

    public SpindexerIOKraken() {
        spindexer = new TalonFX(CAN_ID); // TODO SET VALUE

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake))
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);

        spindexer.getConfigurator().apply(config);

        ControlRequest _threadInterruptStop = new NeutralOut().withUpdateFreqHz(0);
    }

    @Override
    public void setSpeed(double speed) {
        spindexer.set(speed);
    }
}

