package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;


public interface ShooterIO {

    @AutoLog
    public class ShooterInputs {

        public double rpm;
        public double current;

    }

    default void updateInputs() {}

    default void setShooterSpeed(double speed) {}

    default void setFeederSpeed(double speed) {}
     
}