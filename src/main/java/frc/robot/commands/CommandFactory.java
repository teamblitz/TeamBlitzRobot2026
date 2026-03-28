package frc.robot.commands;

import static frc.robot.Constants.Intake.L4_PLOP;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.wrist.Wrist;

public class CommandFactory {
    //Creates the autoShoot command, which can be called during auto
    public static Command autoShoot(Shooter shooter, Spindexer spindexer) {
//        return Commands.parallel(shooter.shootTest(), spindexer.feed());
        return Commands.parallel(
                shooter.feed(),
                Commands.sequence(
                    Commands.waitSeconds(0.2),
                    shooter.shoot(),
                    spindexer.feed())
        );
    }

    //Creates a command that stops the shooter, which can be called during auto
    public static Command shooterStop(Shooter shooter, Spindexer spindexer) {
        return Commands.sequence(
            spindexer.stop(),
            Commands.waitSeconds(0.5),
            shooter.stop()
        );

    }

    //Creates a command that puts the intake down, which can be called during auto
    public static Command intakeDown(Intake intake, Wrist wrist) {
        return Commands.parallel(
            wrist.goToDown(),
            Commands.waitSeconds(0.3),
            intake.forward()

        );

    }

    //Creates a command the puts the intake up, wich can be called during auto 
    public static Command intakeUp(Intake intake, Wrist wrist)  {
        return Commands.parallel(
        intake.stop(),
        wrist.goToIdle()
        );
    }

}
