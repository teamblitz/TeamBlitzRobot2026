package frc.robot.subsystems.index;

import org.littletonrobotics.junction.AutoLog;

public interface IndexIO {

    @AutoLog
    public class IndexInputs {
        public double rpm;
        public double current;

    }

    default void updateInputs(IndexInputs inputs) {}

    default void setSpeed(double speed) {}
}