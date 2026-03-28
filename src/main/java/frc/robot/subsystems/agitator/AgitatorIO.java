package frc.robot.subsystems.agitator;

import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {

    @AutoLog
    public class AgitatorInputs {
        public double rpm;
        public double current;
    }

    default void updateInputs(AgitatorInputs inputs) {}

    default void setSpeed(double speed) {}
}
