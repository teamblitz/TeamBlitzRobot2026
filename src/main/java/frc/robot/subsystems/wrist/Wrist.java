package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends BlitzSubsystem {

  private final WristIO io;
  private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();

  public Wrist(WristIO io) {
    super("Wrist");
    this.io = io;
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);

    // Check for encoder divergence every loop
    // This means something mechanical has gone wrong - a slipping shaft, snapped belt, etc.
    // Stop the wrist if the robot is disabled
    if (DriverStation.isDisabled()) {
      io.stop();
    }
  }

  // --- Commands ---

  public Command setSpeed(double speed) {
    return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
  }

  /**
   * Define a position in radians for the motor to drive towards. Right now it sets a position for
   * both motors
   *
   * @param position the position in radians to drive towards
   * @return the command to tell the motors to drive towards that with motionmagic.
   */
  public Command setPosition(double position) {
    // Clamp our position value so that we aren't setting goals outside of our range
    double clampedPosition =
        MathUtil.clamp(
            position, Math.max(EXTENDED_POS, IDLE_POS), Math.min(EXTENDED_POS, IDLE_POS));
    // Set the motion magic to follow our clamped position
    return run(() -> io.setMotionMagic(clampedPosition));
  }

  // Go to positon commands

  public Command goToIdle() {
    return setPosition(IDLE_POS);
  }

  public Command goToDown() {
    return setPosition(EXTENDED_POS);
  }

  // --- Getters ---

  // Average position of both sides - use for general position checks
  @AutoLogOutput(key = "wrist/position")
  public double getPosition() {
    return inputs.absoluteEncoderPosition;
  }

  @AutoLogOutput(key = "wrist/positionLeft")
  public double getPositionLeft() {
    return inputs.absoluteEncoderPositionLeft;
  }

  @AutoLogOutput(key = "wrist/positionRight")
  public double getPositionRight() {
    return inputs.absoluteEncoderPositionRight;
  }

  @AutoLogOutput(key = "wrist/encoderDelta")
  public double getEncoderDelta() {
    return inputs.encoderDelta;
  }
}
