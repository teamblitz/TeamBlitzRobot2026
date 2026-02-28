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
import frc.robot.Constants.Spindexer;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.drive.range.RangeSensorIOFusion;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.TeleopSwerve;

import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOKraken;


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
    private Drive drive;
    private SwerveModule swerveModule;
    private Vision vision;
    private Intake intake;
    private IntakeIO intakeIO;
    private Shooter shooter;
    private frc.robot.subsystems.spindexer.Spindexer spindexer;
    private SpindexerIO spindexerIO;
    private AutoCommands autoCommands;
    //private DriveCommands driveCommands;
    private Wrist wrist;
    private WristIO wristIO;

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
        drive = new Drive(
                                    new SwerveModuleConfiguration(
                                            SwerveModuleConfiguration.MotorType.KRAKEN,
                                            SwerveModuleConfiguration.MotorType.KRAKEN,
                                            SwerveModuleConfiguration.EncoderType.CANCODER),
                                    Constants.Drive.Mod0.CONSTANTS,
                                    Constants.Drive.Mod1.CONSTANTS,
                                    Constants.Drive.Mod2.CONSTANTS,
                                    Constants.Drive.Mod3.CONSTANTS,
                                    new GyroIOPigeon(),
                                    new RangeSensorIOFusion());

       // driveCommands = new DriveCommands(drive);

        vision = new Vision(drive);

        intakeIO = new IntakeIOKraken();

        spindexerIO = new SpindexerIOKraken();

        intake = new Intake(intakeIO);
        shooter = new Shooter(drive);

        spindexer = new frc.robot.subsystems.spindexer.Spindexer(spindexerIO);

        wrist = new Wrist(wristIO);


    }

    //Creating a new driving system so that our robot understands our joystick and control
    private void setDefaultCommands() {
        drive.setDefaultCommand(
                new TeleopSwerve(
                                drive,
                                OIConstants.Drive.X_TRANSLATION,
                                OIConstants.Drive.Y_TRANSLATION,
                                OIConstants.Drive.ROTATION_SPEED,
                                () -> false,
                                () -> Double.NaN,
                                () -> true)
                        .unless(RobotState::isTest)
                        .until(RobotState::isTest)
                        .withName("TeleopSwerve"));
    }
    //Configures our button bindings to the robot commands.
    private void configureTriggerBindings() {
        OIConstants.Drive.RESET_GYRO.onTrue(Commands.runOnce(drive::zeroGyro));
        //        OIConstants.Drive.X_BREAK.onTrue(drive.park());
        //
        //        OIConstants.Drive.BRAKE.onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        //        OIConstants.Drive.COAST.onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        OIConstants.Intake.REVERSE.whileTrue(intake.reverse()); //left bumper
        OIConstants.Intake.FORWARD.whileTrue(intake.forward()); //right bumper
        OIConstants.Shooter.SHOOT.whileTrue(shooter.shootTest()
                .alongWith(spindexer.feed())); //Left Trigger
        OIConstants.Spindexer.FEED.whileTrue(spindexer.feed()); //y
        
        
        OIConstants.Drive.ALIGN_LEFT.whileTrue(new DeferredCommand(
                () -> drive.driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(
                        PositionConstants.getClosestFace(drive.getPose())[0])),
                Set.of(drive)));

        OIConstants.Drive.ALIGN_RIGHT.whileTrue(new DeferredCommand(
                () -> drive.driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(
                        PositionConstants.getClosestFace(drive.getPose())[1])),
                Set.of(drive)));
    }

    //Configures the FRC dashboard and tells the robot several things:
    //Which alliance, autochoosing, match timer,
    //FYI the actual "Dashboard" is elastic(WPILIB)
    private void configureDashboard() {
       var tab = Shuffleboard.getTab("tuning");

       tab.add(
               "Phoenix SignalLogger",
               runEnd(SignalLogger::start, SignalLogger::stop).ignoringDisable(true));

       tab.add("drive/resetOdometry", Commands.runOnce(() -> drive.resetOdometry(new Pose2d())));

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
                    PositionConstants.getClosestFace(drive.getPose());
                })
                .ignoringDisable(true)
                .onlyIf(Robot::isSimulation)
                .schedule();

    }
    //Configures autonomuous copmmands and autochooser
    private void configureAutonomous() {
        autoChooser = new AutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);

        //EXAMPLE
        // autoChooser.addRoutine("leaveRight", () -> autoCommands.leave("leaveRight"));
    }
    //Configures the Autochooser, which is selected in the dashboard(elastic)
    public Command getAutonomousCommand() {
        Logger.recordOutput("selectedAuto", autoChooser.selectedCommand().getName());
        return Commands.none();
        // return Commands.sequence(
        //                 Commands.runOnce(() -> drive.resetRotation(
        //                         AllianceFlipUtil.shouldFlip()
        //                                 ? Rotation2d.kZero
        //                                 : Rotation2d.k180deg)),
        //                 autoChooser.selectedCommandScheduler())
        //         .withName("Auto Command");
    }
}
