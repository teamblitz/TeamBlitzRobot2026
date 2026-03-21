package frc.robot.commands;

import static frc.robot.Constants.Intake.L4_PLOP;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;

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

}
