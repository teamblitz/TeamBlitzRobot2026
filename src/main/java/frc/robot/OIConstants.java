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


    public static final class Wrist {
        // public static final DoubleSupplier MANUAL =
        //         () -> MathUtil.applyDeadband(-OPERATOR_CONTROLLER.getRightY(), .1);

//        public static final Trigger WRIST_UP = OPERATOR_CONTROLLER.a();
//        public static final Trigger WRIST_DOWN = OPERATOR_CONTROLLER.b();

        public static final Trigger UP = OPERATOR_CONTROLLER.x();
        public static final Trigger DOWN = OPERATOR_CONTROLLER.y();

//        public static final Trigger UP_TEST = OPERATOR_CONTROLLER.a();
}

    public static final class Intake {
        public static final Trigger REVERSE = OPERATOR_CONTROLLER.rightBumper();
        public static final Trigger FORWARD = OPERATOR_CONTROLLER.rightTrigger();
        
    }
}