package frc.robot.subsystems.funnal;

import org.littletonrobotics.junction.AutoLog;

public interface FunnalIO {

    @AutoLog
    public class IntakeInputs {
        public double rpm;
        public double current;
    }

    default void updateInputs(IntakeInputs inputs) {}

    default void setSpeed(double speed) {}
}
