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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.math.AllianceFlipUtil;
import frc.lib.reefscape.ScoringPositions;
import frc.robot.Constants.Spindexer;

import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.Shooter;

import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.agitator.AgitatorIOKraken;

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

    private Shooter shooter;
    private Agitator agitator;
    //private DriveCommands driveCommands;

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
        agitator = new Agitator(new AgitatorIOKraken());




//        autoCommands = new AutoCommands(drive, intake, spindexer, shooter);


    }

    //Creating a new driving system so that our robot understands our joystick and control
    private void setDefaultCommands() {




    }
    //Configures our button bindings to the robot commands.
    private void configureTriggerBindings() {
        OIConstants.Shooter.SHOOT.whileTrue(Commands.parallel(shooter.newShoot(), agitator.run()));
        OIConstants.Shooter.ONESHOOT.whileTrue(Commands.parallel(shooter.newShoot(), agitator.run()));

    }

    //Configures the FRC dashboard and tells the robot several things:
    //Which alliance, autochoosing, match timer,
    //FYI the actual "Dashboard" is elastic(WPILIB)
    private void configureDashboard() {
       var tab = Shuffleboard.getTab("tuning");

       tab.add(
               "Phoenix SignalLogger",
               runEnd(SignalLogger::start, SignalLogger::stop).ignoringDisable(true));


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

        //EXAMPLE
        // autoChooser.addRoutine("leaveRight", () -> autoCommands.leave("leaveRight"));
    }
    //Configures the Autochooser, which is selected in the dashboard(elastic)

}
