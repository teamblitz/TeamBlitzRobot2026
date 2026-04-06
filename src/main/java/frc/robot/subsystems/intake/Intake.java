package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;

public class Intake extends BlitzSubsystem {
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    super("intake");

    this.io = io;
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public Command forward() {
    return runEnd(() -> io.setSpeed(-0.8), () -> io.setSpeed(0));
  }

  public Command reverse() {
    return runEnd(() -> io.setSpeed(-0.65), () -> io.setSpeed(0));
  }

  private Command stop() {
    return runOnce(() -> io.setSpeed(0));
  }

  public Command setSpeed(double speed) {
    return startEnd(() -> io.setSpeed(speed), () -> io.setSpeed(0));
  }
}
