/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ShooterConstants.FEEDER_SPEED;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SPEED;

import choreo.auto.AutoChooser;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.*;
import frc.robot.subsystems.agitator.AgitatorIOKraken;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.agitator.Agitator;

import frc.robot.Constants.*;



import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */

    private Vision vision;
    private Intake intake;
    private IntakeIO intakeIO;
    private Shooter shooter;
    private AutoCommands autoCommands;
    //private DriveCommands driveCommands;
    private Agitator agitator;

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

        shooter = new Shooter(new ShooterIOKraken());


        intakeIO = new IntakeIOKraken();


        intake = new Intake(intakeIO);

        agitator = new Agitator(new AgitatorIOKraken());


    }

    private void setDefaultCommands() {




    }

    private void configureTriggerBindings() {



        /*   Shooter   */
        OIConstants.Shooter.SHOOT.whileTrue(
                Commands.parallel(
                        shooter.defaultShoot(SHOOTER_SPEED, FEEDER_SPEED),
                        agitator.forward(),
                        intake.forward()
                )
        );

    }

    //Configures the FRC dashboard and tells the robot several things:
    //Which alliance, autochoosing, match timer,
    //FYI the actual "Dashboard" is elastic(WPILIB)
    private void configureDashboard() {
       var tab = Shuffleboard.getTab("tuning");

       tab.add(
               "Phoenix SignalLogger",
               runEnd(SignalLogger::start, SignalLogger::stop).ignoringDisable(true));

//       tab.add("drive/resetOdometry", Commands.runOnce(() -> drive.resetOdometry(new Pose2d())));

//        tab.add(
//                "wheel radius characterization",
//                DriveCharacterizationCommands.characterizeWheelDiameter(drive));


//        Commands.run(() -> {
//                    PositionConstants.getClosestFace(drive.getPose());
//                })
//                .ignoringDisable(true)
//                .onlyIf(Robot::isSimulation)
//                .schedule();

    }
    //Configures autonomuous cpmmands and autochooser
    private void configureAutonomous() {
        autoChooser = new AutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);

       

        //EXAMPLE
        // autoChooser.addRoutine("leaveRight", () -> autoCommands.leave("leaveRight"));
    }
    //Configures the Autochooser, which is selected in the dashboard(elastic)
    public Command getAutonomousCommand() {
        Logger.recordOutput("selectedAuto", autoChooser.selectedCommand().getName());
//        return autoChooser.selectedCommandScheduler();


        return Commands.none();
//         return Commands.sequence(
//                         Commands.runOnce(() -> drive.resetRotation(
//                                 AllianceFlipUtil.shouldFlip()
//                                         ? Rotation2d.kZero
//                                         : Rotation2d.k180deg)),
//                         autoChooser.selectedCommandScheduler())
//                 .withName("Auto Command");
    }
}
