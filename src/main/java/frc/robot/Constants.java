// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Set this depending on Robot so it can use the right TunerConstants
  public static final RobotType BotType = RobotType.COMP;

  public static enum RobotType {
    COMP, // Main competiion bot
    ALPHA // testing bot
  }

  public static boolean DISABLE_HAL = false; // "IDK what this does" - Noah

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double LOOP_PERIOD_SEC = frc.robot.Robot.defaultPeriodSecs;

  public static final class WristConstants {
    public static final int MOTOR_ID_LEFT = 20;
    public static final int MOTOR_ID_RIGHT = 18;

    public static final int ABS_ENCODER_ID_LEFT = 21;
    public static final int ABS_ENCODER_ID_RIGHT = 19;

    public static final double MAGNET_OFFSET_LEFT = -0.20; // calibrate per robot
    public static final double MAGNET_OFFSET_RIGHT = -0.8427;

    public static final boolean MOTOR_INVERTED_LEFT = true; // tune per your mechanism
    public static final boolean MOTOR_INVERTED_RIGHT = true;
    public static final boolean ENCODER_INVERTED_LEFT = false;
    public static final boolean ENCODER_INVERTED_RIGHT = false;

    // How far apart the two encoders can read before a fault is triggered (in rotations)
    public static final double ENCODER_DIVERGENCE_THRESHOLD = 0.05;

    // How aggressively to correct divergence during motion - start small and increase carefully
    // Too high causes oscillation, too low means the sides drift apart
    public static final double SYNC_CORRECTION_SCALE = 0.3;

    public static final double ROTOR_TO_SENSOR_RATIO = 36;

    public static final double SOFT_LIMIT_FORWARD = 0.36; // TODO tune using phoenix tuner

    public static final double SOFT_LIMIT_REVERSE = 0; // TODO tune using phoenix tuner

    public static final double TOLERANCE = 0.025; // TODO tune given the play of the mechanism

    public static final double IDLE_POS = 0.0353; // TODO tune using abs position in phoenix tuner

    public static final double EXTENDED_POS = 0.34; // TODO tune using abs position in phoenix tuner

    public static final double KG_POS = 0.275;

    public static final double MAX_VELOCITY = 2; // TODO set

    public static final double MAX_ACCEL = 1; // TODO set

    public static final double CURRENT_LIMIT_WRIST =
        80; // may need to be adjusted for further testing

    // Left
    public static final double LEFT_KP = 10; // TODO tune

    public static final double LEFT_KI = 0; // TODO tune

    public static final double LEFT_KD = 0; // TODO tune

    public static final double LEFT_KG = 0.4; // TODO tune

    public static final double LEFT_KV = 1; // TODO tune

    public static final double LEFT_KS = 0.1; // TODO tune

    // Right
    public static final double RIGHT_KP = 10; // TODO tune

    public static final double RIGHT_KI = 0; // TODO tune

    public static final double RIGHT_KD = 0; // TODO tune

    public static final double RIGHT_KG = 0.4; // TODO tune

    public static final double RIGHT_KV = 1; // TODO tune

    public static final double RIGHT_KS = 0.1; // TODO tune
  }

  public static final class Intake {
    public static final int INTAKE_ID = 17; // TODO set val

    public static final double CURRENT_LIMIT = 120; // TODO set val

    public static final boolean INVERTED = true; // may need to be set to false
  }

  public static final class ShooterConstants {
    public static final int RIGHT_SHOOTER_ID = 13;
    public static final int LEFT_SHOOTER_ID = 14;
    public static final int FEEDER_ID = 15;

    public static final double SHOOTER_HEIGHT = 0.4826;
    public static final double HUB_X =
        (!Constants.DISABLE_HAL
                && DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            ? 182.11
            : 469.11;
    public static final double HUB_Y = 158.84;
    public static final double BASIN_H = 72 - SHOOTER_HEIGHT;
    // TODO set this for the new bot
    public static final double SHOOTER_ANGLE = 70;
    // Format for gear is second/first, so how many rotations of our wheels per rotation of the
    // motor
    public static final double SHOOTER_GEAR = 3 / 1;
    public static final double WHEEL_DIAMETER = 0.1016;

    public static final double RAMP = 0.2;
    public static final double KA = 0.1;

    public static final double SHOOTER_SPEED = 1; // get real val
    public static final double FEEDER_SPEED = 0.2; // get real val
  }

  public static final class AgitartorConstants {
    public static final int AGITATOR_ID = 16;
  }

  public static final class Auto {
    public static final class Timings {}
  }
}
