package frc.robot.subsystems.agitator;

import static frc.robot.Constants.AgitartorConstants.AGITATOR_ID;
import static frc.robot.Constants.Intake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AgitatorIOKraken implements AgitatorIO {
  //Create our agitator motor
  public final TalonFX agitator;

  //Constructor passed in as an argument in robotcontainer
  public AgitatorIOKraken() {
    //Set our motor in code to a TalonFX object that references the motor id
    agitator = new TalonFX(AGITATOR_ID); // TODO SET VALUE

    //Create a new TalonFX configuration(this doesn't hold values, it just changes the values in the motor hardware).
    TalonFXConfiguration config = new TalonFXConfiguration();
    //Set Current limits to prevent taking too much power
    config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);
    
    config.MotorOutput.withNeutralMode((NeutralModeValue.Brake))
        .withInverted(
            INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

    agitator.getConfigurator().apply(config);
  }

  @Override
  public void setSpeed(double speed) {
    agitator.set(speed);
  }

  @Override
  public void updateInputs(AgitatorInputs inputs) {}
}
