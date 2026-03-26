package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class OIConstants {

    public static final double XBOX_STICK_DEADBAND = 0.06;

    public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(1);


    public static final class Shooter {
        public static final Trigger SHOOT = OPERATOR_CONTROLLER.x();

    }

}
