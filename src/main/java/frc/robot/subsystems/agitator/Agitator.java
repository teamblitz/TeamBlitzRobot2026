package frc.robot.subsystems.agitator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;

public class Agitator extends BlitzSubsystem {
  // Initiate our Interface object, since the interface is implemented in our AgitatorIOKraken, this
  // "io" object references our motors
  private final AgitatorIO io;

  // Basic Constructor, passed in in robotcontainer.
  public Agitator(AgitatorIO io) {
    super("agitator");

    this.io = io;
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public Command run() {
    return runEnd(() -> io.setSpeed(0.3), () -> io.setSpeed(0));
  }
}
