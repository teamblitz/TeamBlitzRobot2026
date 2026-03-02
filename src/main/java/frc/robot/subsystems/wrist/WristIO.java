package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

    @AutoLog
    public static class WristInputs {
        public double rpm;
        public double current;
        public double absoluteEncoderPosition;
        public double velocityRadiansPerSecond;
    }

    default void updateInputs(WristInputs inputs) {}

    default void setSpeed(double speed) {}

    default void setMotionMagic(double position) {}

    default void stop() {}
}