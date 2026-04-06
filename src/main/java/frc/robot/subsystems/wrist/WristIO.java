package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public static class WristInputs {

    // Left side
    public double absoluteEncoderPositionLeft;
    public double velocityRadiansPerSecondLeft;
    public double currentLeft;

    // Right side
    public double absoluteEncoderPositionRight;
    public double velocityRadiansPerSecondRight;
    public double currentRight;

    // Derived
    public double absoluteEncoderPosition; // average of both sides, used for position checks
    public double encoderDelta; // divergence between sides, used for fault detection
  }

  default void updateInputs(WristInputs inputs) {}

  default void setSpeed(double speed) {}

  default void setMotionMagic(double position) {}

  default void stop() {}
}
