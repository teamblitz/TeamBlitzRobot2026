package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.Constants.ShooterConstants.HUB_X;
import static frc.robot.Constants.ShooterConstants.HUB_Y;
import static frc.robot.Constants.ShooterConstants.SPEED_TOLERANCE;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final Drive drive;

  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

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
   * @param distance the distance from the exit point of the ball to the target in meters
   * @param angle the release angle of the projectile in radians
   * @param heightGain how much higher the target is than the release point in meters
   * @return the required exit velocity to hit the target in Meters per second
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
   * @param gearRatio the gear ratio from motor to wheel, formatted as Motor Rotations/Wheel
   *     rotations
   * @param wheelDiameter the diameter of the contacting wheel in meters
   * @return the rotations per second, as a double, for the wanted velocity
   */
  public double getRPS(double velocity, double gearRatio, double wheelDiameter) {
    double RPS;
    RPS = (((velocity) * gearRatio) / (Math.PI * wheelDiameter));
    return RPS;
  }

  /**
   * Constructs a velocity voltage object to be sent to a talonFX based off of rotations per second
   *
   * @param RPS the rotations per second wanted of the motor
   * @return the velocityvoltage object to be passed
   */
  public VelocityVoltage getVoltage(double RPS) {
    VelocityVoltage voltage = new VelocityVoltage(RPS).withSlot(0);
    // Dynamically calculate feedforward using the CTRE equation: kS * signum(RPS) + kV * RPS
    // kS is static friction feedforward gain
    // kV is velocity feedforward coefficient
    // They can be found in ShooterIOKraken.java
    voltage = voltage.withFeedForward(io.getFeedForward(RPS));
    // ADD voltage = voltage.withAcceleration(RPS*2); IF THE SHOOTER MOTORS DON't START SPINNING.
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
                            getDistance()
                                + Constants.ShooterConstants.SHOOTER_DISTANCE_OFFSET, // Distance
                            Constants.ShooterConstants.SHOOTER_ANGLE, // Angle
                            Constants.ShooterConstants.BALL_HEIGHT) // Heightgain
                        / Constants.ShooterConstants
                            .SHOOTER_EFFICIENCY, // Account for shooter efficiency by dividing by
                    // our percentage TODO Tune this!!
                    Constants.ShooterConstants.SHOOTER_GEAR, // Gear Ratio
                    Constants.ShooterConstants.WHEEL_DIAMETER)) // Wheel Diameter
        < SPEED_TOLERANCE; // How close we want to be to our speed at minimum
  }

  /**
   * Calculates the distance to the target and runs our shooters at that value
   *
   * @return the command to run the shoot
   */
  public Command aimAndShoot() {
    return sequence(
            run(() ->
                    io.setShooterVoltage(
                        getVoltage(
                            getRPS(
                                getVelocity(
                                        getDistance()
                                            + Constants.ShooterConstants
                                                .SHOOTER_DISTANCE_OFFSET, // Distance
                                        Constants.ShooterConstants.SHOOTER_ANGLE, // Angle
                                        Constants.ShooterConstants.BALL_HEIGHT) // Heightgain
                                    / Constants.ShooterConstants
                                        .SHOOTER_EFFICIENCY, // Account for shooter efficiency by
                                // dividing by our percentage TODO Tune
                                // this!!
                                Constants.ShooterConstants.SHOOTER_GEAR, // Gear ratio
                                Constants.ShooterConstants.WHEEL_DIAMETER)))) // Wheel diameter
                .until(this::isAtTargetSpeed), // REPLACE THIS WITH A NORMAL WAIT IF THE BALL NEVER
            // GETS FEEDED
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
   * @return the needed rotation as a rotation2d
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

  /*
   * Creates a newShoot command. The shooter will increase up to speed, then we wait 2 sec so the
   * drum can get up to speed, and then run the feeder.
   * When you release the button the moters will stop.
   *
   * @return the command
   */
  public Command newShoot() {
    return sequence(
            runOnce(() -> io.setShooterSpeed(0.5)), // 0.18 speed for both for endless cycle
            Commands.waitSeconds(2),
            runOnce(() -> io.setFeederSpeed(0.5)),
            idle())
        .finallyDo(
            () -> {
              io.setShooterSpeed(0);
              io.setFeederSpeed(0);
            });
  }

  /*
   * Creates a shoot command. The shooter sets its speed, and then when released it stops the speed
   *
   * @return the command
   */
  public Command shoot() {
    return sequence(Commands.runOnce(() -> io.setFeederSpeed(0.8)))
        .finallyDo(
            () -> {
              io.setFeederSpeed(0);
            });
  }

  /*
   * Creates a unstick command. The shooter sets the feeder speed, and when the button is released it stops.
   *
   * @return the command
   */
  public Command unstick() {
    return sequence(Commands.runOnce(() -> io.setFeederSpeed(0.8)))
        .finallyDo(
            () -> {
              io.setFeederSpeed(0);
            });
  }

  /*
   * Using the AutoLogOutputs, the value of getRPM is recoded in advantage scope.
   */
  @AutoLogOutput(key = "shooter/RPM")
  public double getRPM() {
    return inputs.rpm;
  }
}
