package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.Constants.ShooterConstants.HUB_X;
import static frc.robot.Constants.ShooterConstants.HUB_Y;
import static frc.robot.Constants.ShooterConstants.SPEED_TOLERANCE;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.*;
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
  }

  /**
   * This method calculates the distance from the robot to the hub
   *
   * @return the distance in meters to the target
   */
  public double getDistance() {
    // pose is just where the robot is at the time
    Pose2d pose = drive.getPose();
    // First part of the equation, calculating distance
    double distance =
        Math.sqrt(
            Math.pow((Constants.ShooterConstants.HUB_X - pose.getX()), 2)
                + Math.pow((Constants.ShooterConstants.HUB_Y - pose.getY()), 2));
    return distance;
  }

  /**
   * This method gets the velocity of our shooter wheels with distance
   *
   * @return
   */
  public double getVelocity(double distance, double angle, double heightGain) {
    double velocity;
    // Second part of the equation, converting distance to required velocty
    velocity =
        Math.sqrt(
            9.81
                * Math.pow(distance, 2)
                / ((distance * Math.sin(angle * 2) - heightGain * (1 + Math.cos(angle * 2)))));
    // System.out.println("velocity: " + velocity);
    return velocity;
  }

  /**
   * Gets the required Rotations per second of a wheel based on a velocity
   *
   * @param velocity the wanted velocity
   * @param gearRatio the gear ratio from motor to wheel
   * @param wheelDiameter the diameter of the contacting wheel
   * @return the rotations per second, as a double, for the wanted velocity
   */
  public double getRPS(double velocity, double gearRatio, double wheelDiameter) {
    double RPS;
    RPS =
        ((velocity)
                / (Math.PI * wheelDiameter) // Acounting for wheel dameter
                / gearRatio)
            + 0.01; // Accounting for the gear ratio so
    // System.out.println("RPS: " + RPS);
    return RPS;
  }

  /**
   * Constructs a velocity voltage object to be sent to a talonFX based off of rotations per second
   *
   * @param RPS the rotations per second wanted
   * @return the velocityvoltage object to be passed
   */
  public VelocityVoltage getVoltage(double RPS) {
    VelocityVoltage voltage = new VelocityVoltage(RPS).withSlot(0);
    voltage = voltage.withAcceleration(RPS / 2);
    voltage = voltage.withFeedForward(5);
    // System.out.println(voltage);
    return voltage;
  }

  /**
   * Checks to see if our shooter wheels are at the target speed
   *
   * @return whether or not we are at our target speed
   */
  public boolean isAtTargetSpeed() {
    return Math.abs(
            io.getShooterRPS()
                - getRPS(
                    getVelocity(
                        getDistance(),
                        Constants.ShooterConstants.SHOOTER_ANGLE,
                        Constants.ShooterConstants.BALL_HEIGHT),
                    Constants.ShooterConstants.SHOOTER_GEAR,
                    Constants.ShooterConstants.WHEEL_DIAMETER))
        < SPEED_TOLERANCE;
  }

  /**
   * Calculates the distance to the target and runs our shooters at that value
   *
   * @return the command to run the shoot
   */
  public Command aimAndShoot() {
    return sequence(
            runOnce(
                () ->
                    io.setShooterVoltage(
                        getVoltage(
                            getRPS(
                                getVelocity(
                                    getDistance()
                                        + Constants.ShooterConstants.SHOOTER_DISTANCE_OFFSET,
                                    Constants.ShooterConstants.SHOOTER_ANGLE,
                                    Constants.ShooterConstants.BALL_HEIGHT),
                                Constants.ShooterConstants.SHOOTER_GEAR,
                                Constants.ShooterConstants.WHEEL_DIAMETER)))),
            waitSeconds(2),
            runOnce(() -> io.setFeederSpeed(0.8)),
            idle())
        .finallyDo(
            () -> {
              io.setShooterSpeed(0);
              io.setFeederSpeed(0);
            });
  }

  /**
   * This command gets the needed rotation of the robot to face a target
   *
   * @param targetX the x coordinate of the target
   * @param targetY the y coordinate of the target
   * @param offset the offset, in radians, to offset the angle by
   * @return what the robot should rotate to as a rotation 2d object
   */
  public Rotation2d getTargetRotation(double targetX, double targetY, double offset) {
    double xDiff = targetX - drive.getPose().getX();
    double yDiff = targetY - drive.getPose().getY();
    Rotation2d rotation = new Rotation2d(Math.atan2(yDiff, xDiff) + offset);
    return rotation;
  }

  /**
   * Gets the rotation of the bot as a rotation 2d based on the hub coordinates
   *
   * @return the needed rotation
   */
  public Rotation2d getRotationToHub() {
    return getTargetRotation(HUB_X, HUB_Y, 0);
  }

  /**
   * Spins the wheels up to a speed so that we don't have to delay too much Only stops when
   * interrupted
   *
   * @return the command
   */
  public Command startSpinUp() {
    return startEnd(() -> io.setShooterSpeed(0.65), () -> io.setShooterSpeed(0));
  }

  public Command newShoot() {
    return sequence(
            runOnce(() -> io.setShooterSpeed(0.6)),
            Commands.waitSeconds(2),
            runOnce(() -> io.setFeederSpeed(0.6)),
            idle())
        .finallyDo(
            () -> {
              io.setShooterSpeed(0);
              io.setFeederSpeed(0);
            });
  }

  public Command shoot() {
    return sequence(Commands.runOnce(() -> io.setFeederSpeed(0.8)))
        .finallyDo(
            () -> {
              io.setFeederSpeed(0);
            });
  }

  public Command unstick() {
    return sequence(Commands.runOnce(() -> io.setFeederSpeed(0.8)))
        .finallyDo(
            () -> {
              io.setFeederSpeed(0);
            });
  }
}
