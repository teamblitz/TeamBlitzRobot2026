package frc.robot.subsystems.wrist;
import frc.robot.Constants.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

    public static class WristInputs {
        //rotations per minute
        public static double rpm;
        //Electrical current, not current position
        public static double current;
        public static double absoluteEncoderPosition;
        public static double velocityRadiansPerSecond;
    }

    default void updateInputs(WristInputs inputs) {}

    default void setSpeed(double speed) {}
}