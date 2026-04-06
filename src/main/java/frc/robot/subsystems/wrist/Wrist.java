package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends BlitzSubsystem {

  private final WristIO io;
  private final WristInputsAutoLogged inputs = new WristInputsAutoLogged();

  private Optional<TrapezoidProfile.State> goal;

  @AutoLogOutput(key = "wrist/divergenceFault")
  private boolean divergenceFault = false;

  public Wrist(WristIO io) {
    super("Wrist");
    this.io = io;
    this.goal = Optional.empty();
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);

    // Check for encoder divergence every loop
    // This means something mechanical has gone wrong - a slipping shaft, snapped belt, etc.
    if (inputs.encoderDelta > ENCODER_DIVERGENCE_THRESHOLD) {
      if (!divergenceFault) {
        DriverStation.reportWarning(
            "[Wrist] Encoder divergence fault! Left: "
                + inputs.absoluteEncoderPositionLeft
                + " Right: "
                + inputs.absoluteEncoderPositionRight
                + " Delta: "
                + inputs.encoderDelta,
            false);
        divergenceFault = true;
      }
      // Stop the wrist and clear the goal when a fault is detected
      // to prevent the motors from stressing the mechanism further
      io.stop();
      goal = Optional.empty();
      return;
    } else {
      divergenceFault = false;
    }

    if (goal.isPresent() && DriverStation.isEnabled()) {
      io.setMotionMagic(goal.get().position);
    }

    if (DriverStation.isDisabled()) {
      goal = Optional.empty();
      io.stop();
    }
  }

  // --- Commands ---

  public Command setSpeed(double speed) {
    return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
  }

  public Command goToIdle() {
    return goToPosition(IDLE_POS);
  }

  public Command goToDown() {
    return goToPosition(EXTENDED_POS);
  }

  public Command goToCenter() {
    return goToPosition(KG_POS);
  }

  // Moves to a position and waits until both sides are near the target
  public Command goToPosition(double position) {
    return followGoal(position)
        .withDeadline(
            Commands.waitUntil(
                () -> {
                  // Both sides must individually be near the target, not just the average
                  // This hopefully ensures the mechanism is actually flat, not one side ahead of
                  // the other
                  boolean leftNear =
                      MathUtil.isNear(position, inputs.absoluteEncoderPositionLeft, TOLERANCE);
                  boolean rightNear =
                      MathUtil.isNear(position, inputs.absoluteEncoderPositionRight, TOLERANCE);
                  boolean arrived = leftNear && rightNear;

                  Logger.recordOutput("wrist/goToPosition/leftNear", leftNear);
                  Logger.recordOutput("wrist/goToPosition/rightNear", rightNear);
                  Logger.recordOutput("wrist/goToPosition/arrived", arrived);

                  return arrived;
                }))
        .withName(logKey + "/goToPosition " + position);
  }

  public Command followGoal(double goalPos) {
    return run(() -> {
          if (this.goal.isEmpty() || this.goal.get().position != goalPos) {
            this.goal =
                Optional.of(
                    new TrapezoidProfile.State(
                        MathUtil.clamp(
                            goalPos,
                            Math.min(EXTENDED_POS, IDLE_POS),
                            Math.max(EXTENDED_POS, IDLE_POS)),
                        0));
          }
        })
        .handleInterrupt(() -> this.goal = Optional.empty());
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

  public boolean hasDivergenceFault() {
    return divergenceFault;
  }
}
