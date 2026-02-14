/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoChooser;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.math.AllianceFlipUtil;
import frc.lib.reefscape.ScoringPositions;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Spindexer.Spindexer;
import frc.robot.subsystems.Spindexer.SpindexerIOKraken;

import org.littletonrobotics.junction.Logger;

import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */
    private Spindexer spindexer;
   // private AutoCommands autoCommands;

    /* ***** --- Autonomous --- ***** */
    private AutoChooser autoChooser;

    public RobotContainer() {
        CameraServer.startAutomaticCapture();

        // Configure Subsystems
        configureSubsystems();
        // Set default commands
        setDefaultCommands();
        // Configure Trigger Bindings
        configureTriggerBindings();
        // Configure Autonomous
        configureAutonomous();

        configureDashboard();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureSubsystems() {

        spindexer = new Spindexer(new SpindexerIOKraken()); //TODO Check this over, not sure if it will work

    }

    private void setDefaultCommands() {

        
    }

    private void configureTriggerBindings() {
        //        OIConstants.Drive.X_BREAK.onTrue(drive.park());
        //
        //        OIConstants.Drive.BRAKE.onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        //        OIConstants.Drive.COAST.onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        OIConstants.Spindexer.FEED.whileTrue(spindexer.feed());
        OIConstants.Intake.REVERSE.whileTrue(spindexer.feed());

    }

    private void configureDashboard() {
//        var tab = Shuffleboard.getTab("tuning");
//
//        tab.add(
//                "Phoenix SignalLogger",
//                runEnd(SignalLogger::start, SignalLogger::stop).ignoringDisable(true));
//
//        tab.add("drive/resetOdometry", Commands.runOnce(() -> drive.resetPose(new Pose2d())));
//
//        tab.add(
//                "wheel radius characterization",
//                DriveCharacterizationCommands.characterizeWheelDiameter(drive));

        new Trigger(() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Blue)
                .onChange(runOnce(() -> {
                            for (ScoringPositions.Branch branch :
                                    ScoringPositions.Branch.values()) {
                                Logger.recordOutput(
                                        "positions/reef/" + branch.name(),
                                        PositionConstants.Reef.SCORING_POSITIONS
                                                .get(branch)
                                                .get());
                            }
                        })
                        .ignoringDisable(true));

        Commands.run(() -> {
                    //PositionConstants.getClosestFace(drive.getPose());
                })
                .ignoringDisable(true)
                .onlyIf(Robot::isSimulation)
                .schedule();
    }

    private void configureAutonomous() {
        autoChooser = new AutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);

       // autoChooser.addRoutine("leaveRight", () -> autoCommands.leave("leaveRight"));
    }
}
