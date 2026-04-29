package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class OIConstants {
  /*ALL KEYBINDS
   * Wrist-Down -> y
   * Spindexer -> Pov_Down
   */

  public static final double XBOX_STICK_DEADBAND = 0.06;

  public static final CommandJoystick DRIVE_CONTROLLER = new CommandJoystick(0);
  public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(1);

  public static final Trigger TELEOP = new Trigger(DriverStation::isTeleop);
  public static final Trigger UNBOUND = new Trigger(() -> false);

  public static final Function<Double, Double> INPUT_CURVE = (x) -> .8 * x + .2 * (x * x * x);
  public static final Function<Double, Double> SPIN_CURVE = (x) -> (x * x * x);

  public static final class Shooter {
    public static final Trigger OPERATOR_SHOOT = OPERATOR_CONTROLLER.leftTrigger();
    public static final Trigger DRIVER_SHOOT = DRIVE_CONTROLLER.button(2);
    public static final Trigger OPERATOR_PREFIRE = OPERATOR_CONTROLLER.rightBumper();
    public static final Trigger OPERATOR_PREFIRESHOOT = OPERATOR_CONTROLLER.rightTrigger();
    public static final Trigger OPERATOR_AIM = OPERATOR_CONTROLLER.leftBumper();
    public static final Trigger OPERATOR_UNSTICK = OPERATOR_CONTROLLER.povDown();
    public static final Trigger SHOOT_TESTING = OPERATOR_CONTROLLER.povRight();

    public static final Trigger AIM_TO_TARGET = OPERATOR_CONTROLLER.rightStick();
  }

  //    public static final class TestMode {
  //        public static final Trigger ZERO_ABS_ENCODERS = UNBOUND;
  //
  //        // TODO, Move these to shuffleboard buttons in their respective dashboards.
  //        public static final class SysId {
  //            public static final class Arm {
  //                public static final Trigger ARM_TEST =
  //                        new Trigger(DriverStation::isTest).and(TEST_CONTROLLER.povLeft());
  //                public static final Trigger QUASISTATIC_FWD =
  // ARM_TEST.and(TEST_CONTROLLER.y());
  //                public static final Trigger QUASISTATIC_REV =
  // ARM_TEST.and(TEST_CONTROLLER.x());
  //                public static final Trigger DYNAMIC_FWD = UNBOUND;
  // ARM_TEST.and(TEST_CONTROLLER.b());
  //               public static final Trigger DYNAMIC_FWD =
  // ARM_TEST.and(TEST_CONTROLLER.b());
  //                public static final Trigger DYNAMIC_REV = ARM_TEST.and(TEST_CONTROLLER.a());
  //            }
  //
  //            public static final class Drive {
  //                public static final Trigger DRIVE_TEST =
  //                        new Trigger(DriverStation::isTest).and(TEST_CONTROLLER.povDown());
  //                public static final Trigger QUASISTATIC_FWD =
  // DRIVE_TEST.and(TEST_CONTROLLER.y());
  //                public static final Trigger QUASISTATIC_REV =
  // DRIVE_TEST.and(TEST_CONTROLLER.x());
  //                public static final Trigger DYNAMIC_FWD = DRIVE_TEST.and(TEST_CONTROLLER.b());
  //                public static final Trigger DYNAMIC_REV = DRIVE_TEST.and(TEST_CONTROLLER.a());
  //            }
  //        }
  //    }

  public static final class Overrides {
    //        private static final ShuffleboardTab TAB = Shuffleboard.getTab("Overrides");

    //        @SuppressWarnings("resource")
    //        public static final BooleanSupplier INTAKE_OVERRIDE =
    //                DashboardHelpers.genericEntrySupplier(
    //                                TAB.add("intake", false)
    //                                        .withWidget(BuiltInWidgets.kBooleanBox)
    //                                        .getEntry(),
    //                                false,
    //                                NetworkTableType.kBoolean)
    //                        ::get;
    //
    //        public static final BooleanSupplier ARM_OVERRIDE =
    //                DashboardHelpers.genericEntrySupplier(
    //                                TAB.add("Arm", false)
    //                                        .withWidget(BuiltInWidgets.kBooleanBox)
    //                                        .getEntry(),
    //                                false,
    //                                NetworkTableType.kBoolean)
    //                        ::get;
  }

  public static final class Intake {
    // public static final Trigger REVERSE = OPERATOR_CONTROLLER.rightBumper();
    public static final Trigger FORWARD = OPERATOR_CONTROLLER.y();
    public static final Trigger FORWARD_DRIVER = DRIVE_CONTROLLER.button(4);
  }

  //   public static final class Spindexer {
  //     public static final Trigger FEED = OPERATOR_CONTROLLER.povDown();

  //     public static final Trigger SHOOT = UNBOUND; // TODO set val
  //   }

  public static final class Wrist {
    public static final DoubleSupplier MANUAL =
        () -> MathUtil.applyDeadband(-OPERATOR_CONTROLLER.getRightY(), .1);

    //        public static final Trigger WRIST_UP = OPERATOR_CONTROLLER.a();
    //        public static final Trigger WRIST_DOWN = OPERATOR_CONTROLLER.b();

    public static final Trigger UP = OPERATOR_CONTROLLER.povLeft();
    public static final Trigger PANIC_UP = OPERATOR_CONTROLLER.povUp();

    //        public static final Trigger UP_TEST = OPERATOR_CONTROLLER.a();
  }

  public static final class Agitator {
    public static final Trigger RUN_AGITATOR = OPERATOR_CONTROLLER.povRight();
  }
}
