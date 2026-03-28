package frc.robot.subsystems.agitator;

import static frc.robot.Constants.Agitator.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

public class AgitatorIOKraken implements AgitatorIO {
    public final TalonFX agitator;

    public AgitatorIOKraken() {
        agitator = new TalonFX(AGITATOR_ID); // TODO SET VALUE

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake))
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);


        agitator.getConfigurator().apply(config);
    }



    @Override
    public void setSpeed(double speed) {
        agitator.set(speed);

    }

    @Override
    public void updateInputs(AgitatorInputs inputs) {


    }

}
