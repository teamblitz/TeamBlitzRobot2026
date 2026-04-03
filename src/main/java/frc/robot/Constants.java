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
  public static final RobotType BotType = RobotType.ALPHA;

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
    public static final int WRIST_ID = 21;
    public static final int ABS_ENCODER_ID = 22;
    //       public static final double WRIST_GEAR_RATIO = (9.0) * (5.0) * (54.0 / 16.0);

    //      public static final double OPEN_LOOP_RAMP = .25;
    public static final int CURRENT_LIMIT_WRIST =
        80; // TODO make this value higher after initial testing
    public static final boolean INVERTED = false;

    // TODO TUNE MAXIMUM AND MINIMUM POSITIONS WHEN WE HAVE THE BOT
    //        public static final double ZERO_POS = Units.degreesToRadians(compBot() ? 94 : 90);
    public static final double IDLE_POS = 0.085;
    //        public static final double EXTENDED_POS = Units.degreesToRadians(-90);//TODO set this
    // val to absolute encoder value when this is extended
    public static final double EXTENDED_POS =
        0.35; // TODO tune on actual bot in reference to the zero position

    public static final double OFFSET = IDLE_POS + 0.25;

    public static final double ABS_ENCODER_ZERO = Math.toRadians(306.71 + 90);
    //        public static final double TOLERANCE = Units.degreesToRadians(10);
    public static final double TOLERANCE = 0.005;

    public static final double MAX_VELOCITY = 2; // TODO tune on actual bot
    public static final double MAX_ACCEL = 42; // TODO tune on actual bot
    public static final double MAX_JERK = MAX_ACCEL * 8;

    public static final double ROTOR_TO_SENSOR_RATIO = 9.0;
    public static final double MAGNET_OFFSET = -0.144775390625; // TODO tune on actual bot

    public static final double KP = 20; // TODO tune KP, KD, KV, and KS on actual bot
    public static final double KI = 0;
    public static final double KD = 1;
    public static final double KG = 1;
    public static final double KV = 0;
    public static final double KS = 0.5;

    public static final double SOFT_LIMIT_FORWARD =
        0.35; // Sets the bounds for where the wrist can go
    public static final double SOFT_LIMIT_REVERSE =
        0.08; // Sets the bounds for where the wrist can go

    public static final class PidGains {
      public static final double KP = 2;
      public static final double KI = 0;
      public static final double KD = 0; // 524.32
    }

    // bad cuz backlash
    public static final class WristGains {
      public static final double KS = 0.0080265;
      public static final double KV = 2.8461;
      public static final double KA = 0.66332;
      public static final double KG = 0.2288;
    }

    public static final class KrakenGains {
      public static final double KS = 0.3576;
      public static final double KV = 2.5726 * (2 * Math.PI);
      public static final double KA = 0.059493 * (2 * Math.PI);
      public static final double KG = 0.16157;
      public static final double KP = 6 * 12 * (2 * Math.PI);
      public static final double KD = 0; // 0.70647
    }
  }

  public static final class Intake {
    public static final int INTAKE_ID = 0; // TODO set val

    public static final double CURRENT_LIMIT = 120; // TODO set val

    public static final boolean INVERTED = true; // may need to be set to false
  }

  public static final class ShooterConstants {
    public static final int RIGHT_SHOOTER_ID = 0; // TODO set val
    public static final int LEFT_SHOOTER_ID = 0; // TODO set val
    public static final int FEEDER_ID = 0; // TODO set val

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

    public static final double RAMP = 0.15;
    public static final double KA = 0.1;

    public static final double SHOOTER_SPEED = 1; // get real val
    public static final double FEEDER_SPEED = 0.2; // get real val
  }

  public static final class Auto {
    public static final class Timings {
      public static final double STOW_TO_L4_READY = 1.25; // IDK ACTUAL TIMINGS
    }
  }
}
