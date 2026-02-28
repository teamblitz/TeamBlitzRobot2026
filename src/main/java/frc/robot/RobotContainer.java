/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.WristConstants.EXTENDED_POS;
import static frc.robot.Constants.WristConstants.ZERO_POS;

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
import frc.robot.Constants.WristConstants;

// import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;

import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOKraken;

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
//     private CommandSwerveDrivetrain drive;

    private Intake intake;

    private Wrist wrist;

    /* ***** --- Autonomous --- ***** */
    private AutoChooser autoChooser;

    public RobotContainer() {
        CameraServer.startAutomaticCapture();

        // Configure Subsystems
        configureSubsystems();
        // Set default commands
        // setDefaultCommands();
        // Configure Trigger Bindings
        configureTriggerBindings();
        // Configure Autonomous
        

        configureDashboard();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureSubsystems() {
        // drive = TunerConstants.createDrivetrain();
        // driveCommands = new DriveCommands(drive);

        // vision = new Vision(drive);

        //intake = new Intake(Constants.compBot() ? new IntakeIOKraken() : new IntakeIOSpark());

        wrist = new Wrist(new WristIOKraken());

    }

// //     private void setDefaultCommands() {
//         drive.setDefaultCommand(driveCommands
//                 .joystickDrive(
//                         OIConstants.Drive.X_TRANSLATION,
//                         OIConstants.Drive.Y_TRANSLATION,
//                         OIConstants.Drive.ROTATION_SPEED,
//                         () -> 5,
//                         () -> 10,
//                         () -> 2 * Math.PI,
//                         true)
//                 .onlyWhile(RobotState::isTeleop)
//                 .onlyIf(RobotState::isTeleop)
//                 .withName("Joystick Drive"));

        
//     }

    private void configureTriggerBindings() {
        // OIConstants.Drive.RESET_GYRO.onTrue(Commands.runOnce(() -> drive.resetRotation(
                // AllianceFlipUtil.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero)));
        //        OIConstants.Drive.X_BREAK.onTrue(drive.park());
        //
        //        OIConstants.Drive.BRAKE.onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        //        OIConstants.Drive.COAST.onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        //Tells Wrist to move to position MAX, or the max position
        //The reason for the .onTrue, is because the trigger is a constant boolean value. This jsut says
        // to only run the cpommand once and not break the bot.
        // OIConstants.Wrist.WRIST_UP.onTrue(Commands.runOnce(() -> {
        //     System.out.println("*************working");
        //     wrist.goToPositon(ZERO_POS);
        // }
        // ));

        OIConstants.Wrist.WRIST_DOWN.onTrue(wrist.goToDown());
        OIConstants.Wrist.WRIST_UP.onTrue(wrist.goToIdle());

        OIConstants.Wrist.MAN_UP.whileTrue(wrist.move_up());
       

       
        // OIConstants.Intake.REVERSE.whileTrue(intake.reverse());
        
        // OIConstants.Drive.ALIGN_LEFT.whileTrue(new DeferredCommand(
        //         () -> drive.driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(
        //                 PositionConstants.getClosestFace(drive.getPose())[0])),
        //         Set.of(drive)));

        // OIConstants.Drive.ALIGN_RIGHT.whileTrue(new DeferredCommand(
        //         () -> drive.driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(
        //                 PositionConstants.getClosestFace(drive.getPose())[1])),
        //         Set.of(drive)));
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
                }

        // // Commands.run(() -> {
        // //             PositionConstants.getClosestFace(drive.getPose());
        // //         })
        //         .ignoringDisable(true)
        //         .onlyIf(Robot::isSimulation)
        //         .schedule();

}