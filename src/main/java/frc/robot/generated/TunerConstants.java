package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

/** Main TunerConstants Switcher so we can have multiple TunerConstants for different bots */
public class TunerConstants {

  public static final CANBus kCANBus =
      Constants.BotType == RobotType.COMP
          ? TunerConstantsComp.kCANBus
          : TunerConstantsAlpha.kCANBus;

  public static final LinearVelocity kSpeedAt12Volts =
      Constants.BotType == RobotType.COMP
          ? TunerConstantsComp.kSpeedAt12Volts
          : TunerConstantsAlpha.kSpeedAt12Volts;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          Constants.BotType == RobotType.COMP
              ? TunerConstantsComp.FrontLeft
              : TunerConstantsAlpha.FrontLeft;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          Constants.BotType == RobotType.COMP
              ? TunerConstantsComp.FrontRight
              : TunerConstantsAlpha.FrontRight;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          Constants.BotType == RobotType.COMP
              ? TunerConstantsComp.BackLeft
              : TunerConstantsAlpha.BackLeft;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          Constants.BotType == RobotType.COMP
              ? TunerConstantsComp.BackRight
              : TunerConstantsAlpha.BackRight;

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      Constants.BotType == RobotType.COMP
          ? TunerConstantsComp.DrivetrainConstants
          : TunerConstantsAlpha.DrivetrainConstants;
}
