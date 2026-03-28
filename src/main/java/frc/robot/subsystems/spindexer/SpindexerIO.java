package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

    @AutoLog
    public class IntakeInputs {
        public double rpm;
        public double current;
    }

    default void updateInputs(IntakeInputs inputs) {}

    default void setSpeed(double speed) {}
}
