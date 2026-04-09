package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.wrist.Wrist;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final Drive drive;

  public Shooter(ShooterIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    super.periodic();
    System.out.println(getDistance());
  }

  /**
   * This method calculates the required velocity for the ball, given the distance to target.
   *
   * @return velocity, the required velocity
   */
  public double getDistance() {
    // pose is just where the robot is at the time
    Pose2d pose = drive.getPose();
    // First part of the equation, calculating distance
    double distance =
        Math.sqrt(
            Math.pow((Constants.ShooterConstants.HUB_X - pose.getX()), 2)
                + Math.pow(
                    (Constants.ShooterConstants.HUB_Y - pose.getY()), 2));
    return Math.abs(distance);
  }

  public double getVelocity() {
    // Second part of the equation, converting distance to required velocty
    double distance = getDistance();
    double velocity;
    velocity =
        Math.sqrt(
            9.81
                    * Math.pow(distance, 2)
                    / distance
                    * Math.sin(Constants.ShooterConstants.SHOOTER_ANGLE * 2)
                - Constants.ShooterConstants.BASIN_H
                    * (1 + Math.cos(Constants.ShooterConstants.SHOOTER_ANGLE * 2)));
    return velocity;
  }

  // Convert velocity to Rotations per second, because talon uses that for some reason
  public double getRPS() {
    double RPS;
    RPS =
        (getVelocity())
            / (Math.PI
                * Constants.ShooterConstants.WHEEL_DIAMETER) // Acounting for wheel circumference
            / Constants.ShooterConstants.SHOOTER_GEAR; // Accounting for the gear ratio

    return RPS;
  }

  public VelocityVoltage getVoltage() {
    System.out.println("Got to Voltage");
    VelocityVoltage voltage = new VelocityVoltage(getRPS());

    return voltage;
  }

  public double basicSpeeds() {
    double speed = 0.58;
    double distance = getDistance();
    if (distance > 50) {
      speed = 0.9;
    } else if (distance < 35) {
      speed = 0.48;
    } else {
      speed = 0.58;
    }
    return speed;
  }

  //  public Command aimAndShoot() {
  //    return runOnce(() -> shooter.setVoltage(getVoltage()))
  //        .andThen(Commands.waitSeconds(1))
  //        .andThen(() -> feeder.set(1))
  //        .andThen(Commands.waitSeconds(5))
  //        .andThen(() -> feeder.set(1))
  //        .andThen(Commands.waitSeconds(1)) // TODO set values to run
  //        .finallyDo(
  //            () -> {
  //              shooter.set(0);
  //              feeder.set(0);
  //            });
  //  }

  //  public Command newShoot() {
  //    return runOnce(() -> io.setShooterSpeed(1))
  //        .andThen(Commands.waitSeconds(1.2))
  //        .andThen(() -> io.setFeederSpeed(1))
  //        .andThen((Commands.waitSeconds(30)))
  //        .finallyDo(
  //            () -> {
  //              io.setShooterSpeed(0);
  //              io.setFeederSpeed(0);
  //            });
  //  }

  public Command newShoot(double speed) {
    return Commands.sequence(
            Commands.runOnce(() -> io.setShooterSpeed(speed)),
            Commands.waitSeconds(2),
            Commands.runOnce(() -> io.setFeederSpeed(0.48)),
            // Commands.runOnce(() -> Wrist.goToIdle()),
            Commands.idle())
        .finallyDo(
            () -> {
              io.setShooterSpeed(0);
              io.setFeederSpeed(0);
            });
  }

  public Command startSpinUp() {
    return Commands.sequence(Commands.runOnce(() -> io.setShooterSpeed(0.58)))
        .finallyDo(
            () -> {
              io.setShooterSpeed(0);
            });
  }

  public Command shoot() {
    return Commands.sequence(Commands.runOnce(() -> io.setFeederSpeed(0.48)))
        .finallyDo(
            () -> {
              io.setFeederSpeed(0);
            });
  }
}
