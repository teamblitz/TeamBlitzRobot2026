package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

    @AutoLog
    public static class WristInputs {
        // Rotations per minute
        public static double rpm;
        // Electrical current, not current position
        public static double current;
        public static double absoluteEncoderPosition;
        public static double velocityRadiansPerSecond;
    }

    default void updateInputs(WristInputs inputs) {}

    default void setSpeed(double speed) {}

    // Position in rotations, cruiseVelocity in rotations per second
    default void setMotionMagic(double position) {}

    default void stop() {}
}
