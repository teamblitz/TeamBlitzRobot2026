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
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.stream.IntStream;

public class AutoCommands {
    private final SwerveDriveKinematics kinematics;
    private final Intake intake;
    private final Shooter shooter;
    private final Agitator agitator;

    //    private final Command configAutonDefault;

    // private SwerveSample lastSample;

    public AutoCommands(Intake intake, Agitator agitator, Shooter shooter) {
        this.intake = intake;
        this.kinematics = Constants.Drive.KINEMATICS;
        this.agitator = agitator;
        this.shooter = shooter;
        
        //Defining autofactory and creating a new autofactory



        // "I heard you liked commands, so I gave you a command to set the default command to be a
        // different command" - Noah 2024
        //
        //So this changes the robot's default command when autonomous is started. 
        //The default command allows you to change different properties on the bot.
        // RobotModeTriggers.autonomous()
        //         .onTrue(Commands.runOnce(() -> drive.setDefaultCommand(Commands.run(
        //                 () -> {
        //                     if (lastSample != null) {
        //                         Logger.recordOutput("drive/auto/doingDefaultPid", Math.random());
        //                         drive.followTrajectory(new SwerveSample(
        //                                 lastSample.t,
        //                                 lastSample.x,
        //                                 lastSample.y,
        //                                 lastSample.heading,
        //                                 0,
        //                                 0,
        //                                 0,
        //                                 0,
        //                                 0,
        //                                 0,
        //                                 lastSample.moduleForcesX(),
        //                                 lastSample.moduleForcesY()));
        //                     }
        //                 },
        //                 drive))));

        // "As funny as it would be for the auto to steal the controls of the robot for the rest of
        // the match,
        // unfortunately that is undesired behavior :(, so we need to give them back" - Noah 2024



    }


}
