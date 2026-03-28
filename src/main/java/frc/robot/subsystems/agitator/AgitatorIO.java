package frc.robot.subsystems.agitator;

import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {

    @AutoLog
    public class IntakeInputs {
        public double rpm;
        public double current;

    }

    default void updateInputs(IntakeInputs inputs) {}

    default void setSpeed(double speed) {}
}
