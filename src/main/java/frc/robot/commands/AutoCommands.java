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
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.stream.IntStream;

public class AutoCommands {
    private final CommandSwerveDrivetrain drive;
    private final SwerveDriveKinematics kinematics;
    private final AutoFactory autoFactory;
    private final Intake intake;

    //    private final Command configAutonDefault;
    private final Command configTeleDefault;

    private SwerveSample lastSample;

    public AutoCommands(
            CommandSwerveDrivetrain drive, Intake intake) {
        this.drive = drive;
        this.intake = intake;
        this.kinematics = Constants.Drive.KINEMATICS;

        autoFactory = new AutoFactory(
                drive::getPose,
                drive::resetPose,
                sample -> {
                    // Don't ask, just cast (the ring into the fire frodo)
                    lastSample = (SwerveSample) sample;
                    drive.followTrajectory((SwerveSample) sample);
                },
                true,
                drive);

        Command normalDriveDefault = drive.getDefaultCommand();

        // I heard you liked commands, so I gave you a command to set the default command to be a
        // different command
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

        // As funny as it would be for the auto to steal the controls of the robot for the rest of
        // the match,
        // unfortunately that is undesired behavior :(, so we need to give them back
        configTeleDefault = Commands.runOnce(() -> drive.setDefaultCommand(normalDriveDefault))
                .ignoringDisable(true);

        RobotModeTriggers.autonomous().onFalse(configTeleDefault);
        RobotModeTriggers.teleop().onTrue(configTeleDefault);
    }

    public AutoFactory getFactory() {
        return autoFactory;
    }

    public Command getNoAuto() {
        final var routine = autoFactory.newRoutine("None");
        routine.active().onTrue(Commands.print("Running No Auto"));

        return routine.cmd();
    }

    public AutoRoutine testDrive() {
        final var routine = autoFactory.newRoutine("test");
        final var traj = routine.trajectory("test");

        routine.active()
                .whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd())
                        .withName("auto/cmdSec"));

        return routine;
    }

    public AutoRoutine leave(String pathName) {
        final var routine = autoFactory.newRoutine(pathName);
        final var traj = routine.trajectory(pathName);

        routine.active()
                .whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd())
                        .withName("auto/cmdSec"));

        return routine;
    }
}
