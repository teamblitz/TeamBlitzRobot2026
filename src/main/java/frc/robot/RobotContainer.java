package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.agitator.AgitatorIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOKraken;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Shooter shooter;
  private final Wrist wrist;
  private final Agitator agitator;

  private final AutoFactory autoFactory;

  // Controller
  //  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick driveController = OIConstants.DRIVE_CONTROLLER;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  //  private final AutoFactory autoFactory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // Real robot, instantiate hardware IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOLimelight(camera0Name, drive::getRotation)
                /*, new VisionIOLimelight(camera1Name, drive::getRotation)*/ );
        intake = new Intake(new IntakeIOKraken());
        shooter = new Shooter(new ShooterIOKraken(), drive);
        wrist = new Wrist(new WristIOKraken());
        agitator = new Agitator(new AgitatorIOKraken());
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        autoFactory =
            new AutoFactory(
                drive::getPose,
                drive::setPose,
                drive::runChoreoTrajectory,
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red,
                drive);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // Sim robot, instantiate physics sim IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {}, drive);
        wrist = new Wrist(new WristIO() {});
        agitator = new Agitator(new AgitatorIOKraken());

        autoFactory =
            new AutoFactory(
                drive::getPose,
                drive::setPose,
                drive::runChoreoTrajectory,
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red,
                drive);

        break;

      default: // REPLAY
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {}, drive);
        wrist = new Wrist(new WristIO() {});
        agitator = new Agitator(new AgitatorIOKraken());

        autoFactory =
            new AutoFactory(
                drive::getPose,
                drive::setPose,
                drive::runChoreoTrajectory,
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red,
                drive);

        break;
    }
    //    autoFactory =
    //        new AutoFactory(
    //            drive::getPose,
    //            drive::setPose,
    //            drive::runChoreoTrajectory,
    //            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
    //                == DriverStation.Alliance.Red,
    //            drive);

    // Set up auto routines
    // Replace with:
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    configureSysId();
    configureSubsystems();
    configureButtonBindings();
  }

  private void configureSysId() {

    /*   Autos   */
    autoChooser.addOption("Straight Test", straightTestAuto());
    autoChooser.addOption("LeftSide", leftSideToCenter());
    autoChooser.addOption("RightSide", rightSideToCenter());
    autoChooser.addOption("Center", centerAuto());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  // Throw default commands here for simplicity’s sake
  private void configureSubsystems() {
    wrist.setDefaultCommand(wrist.goToDown());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getY(),
            () -> -driveController.getX(),
            () -> -driveController.getTwist()));

    // Lock to 0° when trigger is held
    driveController
        .trigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getY(),
                () -> -driveController.getX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when button 2 is pressed
    driveController.button(2).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when button 3 is pressed
    driveController
        .button(5)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Super spin when button 1 is held
    driveController
        .button(1)
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> wrist.followGoal(Constants.WristConstants.IDLE_POS), wrist),
                DriveCommands.joystickDrive(
                    drive,
                    () -> driveController.getY(),
                    () -> driveController.getX(),
                    () -> -0.1)));
    // TODO DRIVERS REVIEW BINDINGS
    // ALL BUTTON BINDINGS
    /*
     * OPERATOR:
     *  Intake:             Y ->
     *      Runs intake and agitator
     *  Spin-Up:            RB ->
     *      Runs only the shooting wheels at speed
     *  Pre-fired Shoot:    RT ->
     *      Runs shooter, feeder, agitator, and intake all at the same time with no wind-up
     *  Shoot:              LT ->
     *      Runs shooter, feeder, agitator, and intake with slight delay on the shooter
     *  Unstick:            V  ->
     *      Runs shooter in reverse
     *  Wrist-Up:           < ->
     *      Brings wrist up and runs agitator
     *  Panic-Up:           ^ ->
     *      Brings wrist up
     *  Agitator:           > ->
     *      runs agitator
     *
     * DRIVER:
     *  Intake:             Button 4 ->
     *      Runs intake and agitator
     *  Shoot:              Button 2 ->
     *      Runs shooter, feeder, agitator, and intake with slight delay on the shooter
     *
     */
    // Intake
    OIConstants.Intake.FORWARD.whileTrue(intake.forward().alongWith(agitator.run()));
    OIConstants.Intake.FORWARD_DRIVER.whileTrue(intake.forward().alongWith(agitator.run()));

    // Shooter
    //    OIConstants.Shooter.OPERATOR_SHOOT.whileTrue(shooter.aimAndShoot());
    //    OIConstants.Shooter.OPERATOR_SHOOT.whileTrue(
    //        shooter.newShoot().alongWith(agitator.run())); // 43.5 inches away for current values
    OIConstants.Shooter.OPERATOR_AIM.whileTrue(
        Commands.parallel(
            shooter.aimAndShoot(), wrist.goToIdle(), agitator.run(), intake.forward()));
    OIConstants.Shooter.OPERATOR_PREFIRE.whileTrue(shooter.startSpinUp());
    OIConstants.Shooter.OPERATOR_UNSTICK.whileTrue(shooter.unstick());
    OIConstants.Intake.OPERATOR_INTAKE_REVERSE.whileTrue(intake.reverse());
    OIConstants.Shooter.SHOOT_TESTING.whileTrue(shooter.shoot());
    OIConstants.Shooter.OPERATOR_PREFIRESHOOT.whileTrue(
        Commands.parallel(
            shooter.startSpinUp(),
            shooter.shoot(),
            wrist.goToIdle(),
            agitator.run(),
            intake.forward()));
    OIConstants.Shooter.OPERATOR_SHOOT.whileTrue(
        Commands.parallel(
            shooter.aimAndShoot(), wrist.goToIdle(), agitator.run(), intake.forward()));

    OIConstants.Shooter.DRIVER_SHOOT.whileTrue(
        Commands.parallel(shooter.newShoot(), wrist.goToIdle(), agitator.run(), intake.forward()));
    // Wrist
    //
    // OIConstants.Wrist.PANIC_UP.whileTrue(intake.forwardWithWrist().alongWith(wrist.goToIdle()));
    OIConstants.Wrist.UP.whileTrue(
        Commands.parallel(wrist.goToIdle(), intake.forwardWithWrist(), agitator.run()));

    OIConstants.Wrist.PANIC_UP.whileTrue(wrist.followGoal(Constants.WristConstants.IDLE_POS));

    // Agitator
    OIConstants.Agitator.RUN_AGITATOR.whileTrue(agitator.run());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Command straightTestAuto() {
    AutoRoutine routine = autoFactory.newRoutine("StraightTest");
    AutoTrajectory path = routine.trajectory("StraightTest");

    routine.active().onTrue(Commands.sequence(path.resetOdometry(), path.cmd()));

    return routine.cmd();
  }

  // ***IMPORTANT: Need to set up event markers in the choreo path
  private Command leftSideToCenter() {
    AutoRoutine routine = autoFactory.newRoutine("LeftSideAuto");
    AutoTrajectory path = routine.trajectory("LeftSideAuto");

    routine.active().onTrue(Commands.sequence(path.resetOdometry(), path.cmd()));

    // Runs intake and agitator for optimal hopper usage. Wrist is set to default down
    path.active().whileTrue(intake.forward().alongWith(agitator.run()));

    // Pre-spin shooter near end of path. Need an event marker named "spinup" in Choreo.
    // newShoot will cancel this naturally when doneFor fires since both require Shooter
    path.atTime("spinup").onTrue(shooter.startSpinUp());

    /*  Runs the shoot command
     *  Sets the shooter speed for 2 seconds. After 2 seconds runs the feeder
     *  Runs the agitator
     *  Brings the wrist to idle for any balls stuck in the intake
     *  4 seconds: 2s for shooter to confirm speed + 1.5s feeding + 0.5s buffer
     */
    path.doneFor(1)
        .whileTrue(shooter.newShoot().alongWith(agitator.run()).alongWith(wrist.goToIdle()));

    return routine.cmd();
  }

  private Command rightSideToCenter() {
    AutoRoutine routine = autoFactory.newRoutine("RightSide");
    AutoTrajectory path = routine.trajectory("RightSideAuto");

    routine.active().onTrue(Commands.sequence(path.resetOdometry(), path.cmd()));

    // Runs intake and agitator for optimal hopper usage. Wrist is set to default down
    path.active().whileTrue(intake.forward().alongWith(agitator.run()));

    // Pre-spin shooter near end of path. Need an event marker named "spinup" in Choreo.
    // newShoot will cancel this naturally when doneFor fires since both require Shooter
    path.atTime("spinup").onTrue(shooter.startSpinUp());

    /*  Runs the shoot command
     *  Sets the shooter speed for 2 seconds. After 2 seconds runs the feeder
     *  Runs the agitator
     *  Brings the wrist to idle for any balls stuck in the intake
     *  4 seconds: 2s for shooter to confirm speed + 1.5s feeding + 0.5s buffer
     */
    path.doneFor(1)
        .whileTrue(shooter.newShoot().alongWith(agitator.run()).alongWith(wrist.goToIdle()));

    return routine.cmd();
  }

  private Command centerAuto() {
    AutoRoutine routine = autoFactory.newRoutine("CenterAuto");
    AutoTrajectory path = routine.trajectory("CenterAuto");

    path.atTime("spinup").onTrue(shooter.startSpinUp());

    path.doneFor(1)
        .whileTrue(
            shooter
                .newShoot()
                .alongWith(agitator.run())
                .alongWith(wrist.goToIdle())
                .alongWith(intake.forward()));

    return routine.cmd();
  }
}
