package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;

/** Main TunerConstants Switcher so we can have multiple TunerConstants for different bots */
public class TunerConstants {

  // Set this depending on Robot so it can use the right TunerConstants
  public static final RobotType Bot = RobotType.ALPHA;

  public enum RobotType {
    COMP,
    ALPHA
  }

  public static final CANBus kCANBus =
      Bot == RobotType.COMP ? TunerConstantsComp.kCANBus : TunerConstantsAlpha.kCANBus;

  public static final LinearVelocity kSpeedAt12Volts =
      Bot == RobotType.COMP
          ? TunerConstantsComp.kSpeedAt12Volts
          : TunerConstantsAlpha.kSpeedAt12Volts;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          Bot == RobotType.COMP ? TunerConstantsComp.FrontLeft : TunerConstantsAlpha.FrontLeft;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          Bot == RobotType.COMP ? TunerConstantsComp.FrontRight : TunerConstantsAlpha.FrontRight;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft = Bot == RobotType.COMP ? TunerConstantsComp.BackLeft : TunerConstantsAlpha.BackLeft;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          Bot == RobotType.COMP ? TunerConstantsComp.BackRight : TunerConstantsAlpha.BackRight;

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      Bot == RobotType.COMP
          ? TunerConstantsComp.DrivetrainConstants
          : TunerConstantsAlpha.DrivetrainConstants;
}
