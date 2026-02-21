package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface WristIO {

    @AutoLog
    public static class WristInputs {
        public double rpm;
        public double currentAmps;
        public double absoluteEncoderPosition;
        public double velocityRadiansPerSecond;
    }

    public default void updateInputs(WristInputs inputs) {}
    public default void setSpeed(double speed) {}
    public default void setPosition(double positionRotations) {}
}