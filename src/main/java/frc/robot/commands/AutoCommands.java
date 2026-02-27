package frc.robot.commands;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.lib.reefscape.ScoringPositions;
import frc.lib.reefscape.ScoringPositions.Branch;
import frc.robot.Constants;
import frc.robot.PositionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.shooter.Shooter;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.stream.IntStream;

public class AutoCommands {
    private final Drive drive;
    private final SwerveDriveKinematics kinematics;
    private final AutoFactory autoFactory;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Shooter shooter;

    //    private final Command configAutonDefault;
    private final Command configTeleDefault;

    private SwerveSample lastSample;

    public AutoCommands(
            Drive drive, Intake intake, Spindexer spindexer, Shooter shooter) {
        this.drive = drive;
        this.intake = intake;
        this.kinematics = Constants.Drive.KINEMATICS;
        this.spindexer = spindexer;
        this.shooter = shooter;
        
        //Defining autofactory and creating a new autofactory
        autoFactory = new AutoFactory(
                //Getting the current pose(position and rotation)
                drive::getPose,
                drive::resetOdometry,
                sample -> {
                    // "Don't ask, just cast (the ring into the fire frodo)" - Noah 2024
                    //
                    //Telling the bot to follow the trajectory(which is made in choreo)
                    lastSample = (SwerveSample) sample;
                    drive.followTrajectory((SwerveSample) sample);
                },
                true,
                drive);

        Command normalDriveDefault = drive.getDefaultCommand();

        // "I heard you liked commands, so I gave you a command to set the default command to be a
        // different command" - Noah 2024
        //
        //So this changes the robot's default command when autonomous is started. 
        //The default command allows you to change different properties on the bot.
        RobotModeTriggers.autonomous()
                .onTrue(Commands.runOnce(() -> drive.setDefaultCommand(Commands.run(
                        () -> {
                            if (lastSample != null) {
                                Logger.recordOutput("drive/auto/doingDefaultPid", Math.random());
                                drive.followTrajectory(new SwerveSample(
                                        lastSample.t,
                                        lastSample.x,
                                        lastSample.y,
                                        lastSample.heading,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        lastSample.moduleForcesX(),
                                        lastSample.moduleForcesY()));
                            }
                        },
                        drive))));

        // "As funny as it would be for the auto to steal the controls of the robot for the rest of
        // the match,
        // unfortunately that is undesired behavior :(, so we need to give them back" - Noah 2024

        // This reinitializes teleoperated controls for when autonomous ends
        configTeleDefault = Commands.runOnce(() -> drive.setDefaultCommand(normalDriveDefault))
                .ignoringDisable(true);

        RobotModeTriggers.autonomous().onFalse(configTeleDefault);
        RobotModeTriggers.teleop().onTrue(configTeleDefault);
    }

    //Returns our autoFactory field. Containas pose, and the choreo trajectory
    public AutoFactory getFactory() {
        return autoFactory;
    }

    //Creates an auto which does nothing
    public Command getNoAuto() {
        final var routine = autoFactory.newRoutine("None");
        routine.active().onTrue(Commands.print("Running No Auto"));

        return routine.cmd();
    }

    //Creates a test drive auto routine
    public AutoRoutine testDrive() {
        final var routine = autoFactory.newRoutine("test");
        final var traj = routine.trajectory("test");

        routine.active()
                .whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd())
                        .withName("auto/cmdSec"));

        return routine;
    }
    //Creates a new ROUTINE which calls the autoshoot command.
    //It must be done in this way using the commandfactory structure because of java syntax.
    public AutoRoutine autoShoot() { 
        final var routine = autoFactory.newRoutine("Shoot");
        routine.active().whileTrue(CommandFactory.autoShoot(shooter, spindexer));
        return routine;
    }

    // public AutoRoutine driveAway (String pathName) {
    //     final var routine = autoFactory.newRoutine("Drive Away");

    //     drive.followTrajectory(lastSample);

    //     return routine;
    // }




    /**"
     * I channelled my inner AP CSA here, I haven't touched normal for loops in a long time, and it
     * really shows.
     *
     * @param numberOfCoral bingus
     * @param pathName bongus
     * @return boingus
     " - Noah 2024*/
    // public AutoRoutine leave(String pathName) {
    //     final var routine = autoFactory.newRoutine(pathName);
    //     final var traj = routine.trajectory(pathName);

    //     routine.active()
    //             .whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd())
    //                     .withName("auto/cmdSec"));

    //     return routine;
    // }
}
