package frc.robot.subsystems.index;

import static frc.robot.Constants.Intake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

public class IndexIOKraken implements IndexIO {
    public final TalonFX spinDex;
    //public final TalonFX intakeup;

    public IndexIOKraken() {
        spinDex = new TalonFX(INDEX_MOTOR_ID); // TODO SET VALUE
        //intakeup = new TalonFX(ANGLEMOTOR_ID); 

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake))
                .withInverted(
                        INVERTED
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);
        

        spinDex.getConfigurator().apply(config);
        //intakeup.getConfigurator().apply(config);
    }
        
    

    @Override
    public void setSpeed(double speed) {
        spinDex.set(speed);
        //intakeup.set(speed);

    }

    @Override
    public void updateInputs(IntakeInputs inputs) {


    }

}