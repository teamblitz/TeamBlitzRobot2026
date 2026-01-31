package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

public class IntakeIOKraken implements IntakeIO {
    public final TalonFX grabball;
    public final TalonFX intakeup;

    public IntakeIOKraken() {
        grabball = new TalonFX(INTAKEMOTOR_ID); // TODO SET VALUE
        intakeup = new TalonFX(ANGLEMOTOR_ID); 

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake))
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);
        

        grabball.getConfigurator().apply(config);
        intakeup.getConfigurator().apply(config);
    }
        
    

    @Override
    public void setSpeed(double speed) {
        grabball.set(speed);
        intakeup.set(speed);

    }

    @Override
    public void updateInputs(IntakeInputs inputs) {


    }

}
