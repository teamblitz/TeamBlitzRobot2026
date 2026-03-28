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
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SysIdSwerveSteerGains;

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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.reefscape.ScoringPositions;
import frc.robot.Constants.Spindexer;
import frc.robot.commands.*;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.Shooter;

import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOKraken;


import frc.robot.subsystems.wrist.WristIOKraken;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */
    private Intake intake;
    private IntakeIO intakeIO;
    private Shooter shooter;
    private frc.robot.subsystems.spindexer.Spindexer spindexer;
    private SpindexerIO spindexerIO;
    private AutoCommands autoCommands;
    private DriveCommands driveCommands;
    private Wrist wrist;

    /* ***** --- Autonomous --- ***** */
    private AutoChooser autoChooser;

        /* ***** --- Drive --- ***** */
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick joystick = new CommandJoystick(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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
        // drivetrain = TunerConstants.createDrivetrain();

//        RobotModeTriggers.teleop().onTrue(Commands.runOnce(
//                () -> {drive.getCurrentCommand().cancel();}
//        ).ignoringDisable(true));
        driveCommands = new DriveCommands(drivetrain);

        intakeIO = new IntakeIOKraken();

        spindexerIO = new SpindexerIOKraken();

        intake = new Intake(intakeIO);
        shooter = new Shooter(new ShooterIOKraken(), drivetrain);

        spindexer = new frc.robot.subsystems.spindexer.Spindexer(spindexerIO);

        wrist = new Wrist(new WristIOKraken());

//        autoCommands = new AutoCommands(drive, intake, spindexer, shooter);


    }

    //Creating a new driving system so that our robot understands our joystick and control
    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getY() * MaxSpeed)
                .withVelocityY(-joystick.getX() * MaxSpeed)
                .withRotationalRate(-joystick.getTwist() * MaxAngularRate)
         )
        );

        wrist.setDefaultCommand((wrist.goToIdle()));



    }
    //Configures our button bindings to the robot commands.
    private void configureTriggerBindings() {
        OIConstants.Drive.RESET_GYRO.onTrue(Commands.runOnce(() -> drivetrain.resetRotation(
                AllianceFlipUtil.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero)));
        //        OIConstants.Drive.X_BREAK.onTrue(drive.park());
        //
        //        OIConstants.Drive.BRAKE.onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        //        OIConstants.Drive.COAST.onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        OIConstants.Intake.REVERSE.whileTrue(intake.reverse()); //left bumper
        OIConstants.Intake.FORWARD.whileTrue(intake.forward()); //right bumper
        // OIConstants.Shooter.SHOOT.whileTrue(shooter.shootTest()
        //         .alongWith(spindexer.feed())); //Left Trigger
        OIConstants.Shooter.OPERATOR_SHOOT.whileTrue(shooter.feed());

        OIConstants.Shooter.SHOOT_TESTING.whileTrue(
                Commands.sequence(
                        shooter.aimAndShoot()
                        .alongWith(Commands.waitSeconds(0.65)).andThen(spindexer.feed())
                )
        );
        OIConstants.Shooter.DRIVER_SHOOT.whileTrue(
            Commands.sequence(
                shooter.shoot()
                    .alongWith(Commands.waitSeconds(0.1).andThen(spindexer.feed()))
            )
        );
        OIConstants.Shooter.OPERATOR_TEAM_FEED.whileTrue(
                Commands.sequence(
                shooter.teamFeed()
                    .alongWith(Commands.waitSeconds(0.65).andThen(spindexer.feed()))
            ));
            
        OIConstants.Shooter.DRIVER_TEAM_FEED.whileTrue(
                Commands.sequence(
                shooter.teamFeed()
                    .alongWith(Commands.waitSeconds(0.65).andThen(spindexer.feed()))
            ));

        OIConstants.Shooter.DRIVE_DEEP.whileTrue(
                Commands.sequence(
                shooter.deepFeed()
                    .alongWith(Commands.waitSeconds(0.8).andThen(spindexer.feed()))
        ));

        OIConstants.Shooter.OPERATOR_DEEP.whileTrue(
                Commands.sequence(
                shooter.deepFeed()
                    .alongWith(Commands.waitSeconds(0.8).andThen(spindexer.feed()))
        ));

        OIConstants.Spindexer.FEED.whileTrue(spindexer.reverse()); //y
        
         // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.button(10).whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.button(12).whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        //joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // OIConstants.Drive.ALIGN_LEFT.whileTrue(new DeferredCommand(
        //         () -> drive.driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(
        //                 PositionConstants.getClosestFace(drive.getPose())[0])),
        //         Set.of(drive)));

        // OIConstants.Drive.ALIGN_RIGHT.whileTrue(new DeferredCommand(
        //         () -> drive.driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(
        //                 PositionConstants.getClosestFace(drive.getPose())[1])),
        //         Set.of(drive)));

        OIConstants.Wrist.DOWN.whileTrue(
                Commands.parallel(
                        wrist.goToDown(),
                        intake.forward()
                )
        ); //

        OIConstants.Intake.FORWARD.whileTrue(intake.forward()); // a
        OIConstants.Intake.REVERSE.whileTrue(intake.reverse()); // b

        //Sys id tests
        //Calls premade commands generated by Tuner X
        OIConstants.Drive.SYS_ID_QUASISTATIC.whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        OIConstants.Drive.SYS_ID_DYNAMIC.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        OIConstants.Drive.SYS_ID_QUASISTATIC_REVERSE.whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        OIConstants.Drive.SYS_ID_DYNAMIC_REVERSE.whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));        
    }

    //Configures the FRC dashboard and tells the robot several things:
    //Which alliance, autochoosing, match timer,
    //FYI the actual "Dashboard" is elastic(WPILIB)
    private void configureDashboard() {
       var tab = Shuffleboard.getTab("tuning");

       tab.add(
               "Phoenix SignalLogger",
               runEnd(SignalLogger::start, SignalLogger::stop).ignoringDisable(true));

       //tab.add("drive/resetOdometry", Commands.runOnce(() -> drive.resetPose(new Pose2d())));

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

//        Commands.run(() -> {
//                    PositionConstants.getClosestFace(drive.getPose());
//                })
//                .ignoringDisable(true)
//                .onlyIf(Robot::isSimulation)
//                .schedule();

    }
    //Configures autonomuous cpmmands and autochooser
    private void configureAutonomous() {
        autoCommands = new AutoCommands(drivetrain, intake, spindexer, shooter, wrist);
        autoChooser = new AutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);

        autoChooser.addCmd("None", autoCommands::getNoAuto);
        // autoChooser.addRoutine("Shoot Only", autoCommands::autoShoot);
        //
        autoChooser.addRoutine("MoveAndShoot", autoCommands::moveAndShoot);

        autoChooser.addRoutine("RightLeave", autoCommands::leaveRight);
        autoChooser.addRoutine("LeftLeave", autoCommands::leaveLeft);

        //TODO Needs to be Tested
        autoChooser.addRoutine("leftShoot", autoCommands::leftShoot);
        autoChooser.addRoutine("rightShoot", autoCommands::rightShoot);

        //TODO Needs to be Tested
        autoChooser.addRoutine("leftSingleReload",autoCommands::leftSingleReload);

        //EXAMPLE
        // autoChooser.addRoutine("leaveRight", () -> autoCommands.leave("leaveRight"));
    }
    //Configures the Autochooser, which is selected in the dashboard(elastic)
    public Command getAutonomousCommand() {
        Logger.recordOutput("selectedAuto", autoChooser.selectedCommand().getName());
//        return autoChooser.selectedCommandScheduler();

        // return Commands.sequence(
        //         Commands.runOnce(() -> drive.setGyro(AllianceFlipUtil.shouldFlip() ? 0 : 180)),
        //         autoChooser.selectedCommandScheduler()).withName("Auto Command");
        return Commands.sequence(
                        Commands.runOnce(() ->drivetrain.resetRotation(
                                AllianceFlipUtil.shouldFlip()
                                        ? Rotation2d.kZero
                                        : Rotation2d.k180deg)),
                        autoChooser.selectedCommandScheduler())
                .withName("Auto Command");
    }
}
