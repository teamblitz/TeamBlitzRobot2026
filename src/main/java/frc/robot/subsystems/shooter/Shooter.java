package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.drive.Drive;

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
    getDistance();
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
                + Math.pow((Constants.ShooterConstants.HUB_Y - pose.getY()), 2));
    System.out.println("METERS: " + distance);
    System.out.println("INCHES: " + Units.metersToInches(distance));
    return distance;
  }

  public double getVelocity() {
    double velocity;
    double distance = getDistance();
    // Second part of the equation, converting distance to required velocty
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
    System.out.println("Got to RPS");
    double RPS;
    RPS =
        (getVelocity())
            / (Math.PI * Constants.ShooterConstants.WHEEL_DIAMETER) // Acounting for wheel dameter
            / Constants.ShooterConstants.SHOOTER_GEAR; // Accounting for the gear ratio so

    return RPS;
  }

  public VelocityVoltage getVoltage() {
    System.out.println("Got to Voltage");
    VelocityVoltage voltage = new VelocityVoltage(getRPS());

    return voltage;
  }

  public double basicSpeeds() {
    double speed = 0.58;
    double distanceInches = Units.metersToInches(getDistance());
    if (distanceInches > 50) {
      speed = 0.9;
    } else if (distanceInches < 35) {
      speed = 0.48;
    } else {
      speed = 0.58;
    }
    return speed;
  }

  public Command aimAndShoot() {
    return sequence(
            runOnce(() -> io.setShooterVoltage(getVoltage())),
            waitSeconds(2),
            runOnce(() -> io.setFeederSpeed(0.48)),
            idle())
        .finallyDo(
            () -> {
              io.setShooterSpeed(0);
              io.setFeederSpeed(0);
            });
  }

  // Using for auto. This command sets the speed when it starts and only stops when the command is
  // interrupted.
  public Command startSpinUp() {
    return startEnd(() -> io.setShooterSpeed(0.58), () -> io.setShooterSpeed(0));
  }

  public Command newShoot(double speed) {
    return sequence(
            runOnce(() -> io.setShooterSpeed(speed)),
            waitSeconds(2),
            runOnce(() -> io.setFeederSpeed(0.48)),
            idle())
        .finallyDo(
            () -> {
              io.setShooterSpeed(0);
              io.setFeederSpeed(0);
            });
  }

  public Command shoot() {
    return sequence(Commands.runOnce(() -> io.setFeederSpeed(0.48)))
        .finallyDo(
            () -> {
              io.setFeederSpeed(0);
            });
  }
}
