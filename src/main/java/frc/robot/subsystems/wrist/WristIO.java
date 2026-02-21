package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

    @AutoLog
    public static class WristInputs {
        public double rpm;
        public double currentAmps;
        public double absoluteEncoderPosition;
        public double velocityRadiansPerSecond;
    }

    default void updateInputs(WristInputs inputs) {}
    default void setSpeed(double speed) {}
    default void setPosition(double positionRotations) {}
}