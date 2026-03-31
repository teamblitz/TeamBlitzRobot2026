package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.agitator.Agitator;

public class CommandFactory {
    //Creates the autoShoot command, which can be called during auto
    public static Command autoShoot(Shooter shooter, Agitator agitator) {
        return null; //TODO setup for new bot
    }

}
