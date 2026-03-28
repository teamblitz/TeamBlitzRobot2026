/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.lib.util.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.math.AllianceFlipUtil;

import frc.lib.util.COTSSwerveConstants;

import java.util.Arrays;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>Units: Unless defined otherwise, or wrapped with the wpilib units library, all values should
 * be in <a href="https://en.wikipedia.org/wiki/International_System_of_Units">SI Units</a>
 */
public final class Constants {
    public static final Mode SIM_MODE = Mode.SIM;

    public static final boolean TUNING_MODE = true;
    public static boolean DISABLE_HAL = false; // IDK What this does

    public enum Mode {
        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public enum Robot {
        CompBot,
        DevBot,
        SimBot
    }

//     public static final Robot ROBOT = frc.robot.Robot.isReal() ? Robot.CompBot : Robot.SimBot;
        public static final Robot ROBOT = Robot.CompBot;

    public static boolean compBot() {
        // return ROBOT == Robot.CompBot;
        return true;
    }

    public static boolean devBot() {
        return ROBOT == Robot.DevBot;
    }

    //    public static H compDev<H>(H comp, H dev) {
    //        return ROBOT == Robot.CompBot ? comp : dev;
    //    }

    public static final double LOOP_PERIOD_SEC = frc.robot.Robot.defaultPeriodSecs;




    public static final class Intake {
        public static final int INTAKEMOTOR_ID = 0;
        public static final boolean INVERTED = compBot() ? true : false;
        public static final int CURRENT_LIMIT = compBot() ? 80 : 25;
        public static final double HANDOFF_SPEED = compBot() ? .8 : .5;
        public static final double REVERSE_SPEED = -.15;

        public static final double ALGAE_HOLD = -.4; // TODO CONFIG
        public static final double ALGAE_EJECT = .4; // TODO CONFIG

        public static final double ALGAE_REMOVAL = .5; // TODO CONFIG
        public static final double SHOOT_CORAL = .5;
        public static final double L1 = .3;

        public static final double L4_PLOP = .4;

        public static final double INTAKE_SPEED = 0.1;
    }

    public static final class ShooterConstants {
        public static final int RIGHT_SHOOTER_ID = 1;
        public static final int LEFT_SHOOTER_ID = 2;
        public static final int FEEDER_ID = 3;

        public static final double SHOOTER_SPEED = 0.1;
        public static final double FEEDER_SPEED = 0.1;

        public static final double SHOOTER_HEIGHT = 0.4826;
        public static final double HUB_X = (!Constants.DISABLE_HAL && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
         ? 182.11 : 469.11;
        public static final double HUB_Y = 158.84;
        public static final double BASIN_H = 72 - SHOOTER_HEIGHT;
        public static final double SHOOTER_ANGLE = 70;
        public static final double WHEEL_DIAMETER = 0.1016;

        public static final double RAMP = 0.15;
        public static final double KA = 0.1;
    }

    public static final class Agitator {
        public static final int AGITATOR_ID = 4; //TODO set val
        public static final double CURRENT_LIMIT = 60; //TODO set val
        public static final boolean INVERTED = true; //TODO get real val

        public static final double AGITATOR_SPEED = 0.1;
    }


}