package frc.robot.subsystems;


public interface WristIO {

    public class WristInputs {
        public double rpm;
        public double current;

    }

    default void updateInputs(WristInputs inputs) {}

    default void setSpeed(double speed) {}
}