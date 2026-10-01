package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  /*
   * Using the @AutoLog we log the values of RPM and Current to the Shooter Class.
   */
  @AutoLog
  public class ShooterInputs {

    public double rpm;
    public double current;
  }

  // Creates a void called updateInputs that takes values in ShooterIOKranken.
  default void updateInputs() {}

  // Creats a void called setShooterSpeed, that alows the setting of speed to just the shooter.
  default void setShooterSpeed(double speed) {}

  // Creates a void called setFeederSpeed, that alows the setting of speed to just the feeder.
  default void setFeederSpeed(double speed) {}

  // Creates a void called setShooterVoltage that alows the seting of voltage to shooter moters.
  default void setShooterVoltage(VelocityVoltage voltage) {}

  // Creates a double called getShooterRPS, that gets the RPS of the shooter moter.
  default double getShooterRPS() {
    return 0.0;
  }
}
