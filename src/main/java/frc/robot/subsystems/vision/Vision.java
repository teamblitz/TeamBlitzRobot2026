package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
    // Creates poseEstimator 
    public static SwerveDrivePoseEstimator poseEstimator;
    private static CommandSwerveDrivetrain drive;   

    //TODO see if we need these values
    double tx = LimelightHelpers.getTX("limelight");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("limelight");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("limelight");  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV("limelight"); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC("limelight");  // Vertical offset from principal pixel/point to target in degrees

public void init() {
//TODO set the crop window
// Sets a crop window to imporve perfomance (Values between -1 and 1)
LimelightHelpers.setCropWindow("limelight", -1, 1, -1, 1);

// Swiches the limelight to pipline one when in init
LimelightHelpers.setPipelineIndex("limelight", 0);
}

    @Override
    public void periodic() {
        super.periodic();
        // Runs the the getVisionPose periodicly when the robot is enabled 
        getVisionPose();
        //TODO fix this issue. Find swerve postition. 
        poseEstimator.update(drive.getPigeon2().getYaw().getValueAsDouble(), );
    }

/*TODO Look at limelight docs for info on this.) 
 Look for Robot Localization with MegaTag2 and
 FRC Programming with LimelightLib (WPILib Java & C++) */

    public void getVisionPose() {
        // Gets the robots yaw (Degrees) baised on the Pigeon2
        double robotYaw = drive.getPigeon2().getYaw().getValueAsDouble();
        // Sets the robots orintation values to 0 when in starting postion
       LimelightHelpers.SetRobotOrientation("limelight", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        //Gets the pose estimate baised on MegaTag2
        LimelightHelpers.PoseEstimate limelightMeasurement = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        //Estimates whare we are on the field baised on the tag data
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
    poseEstimator.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds
        );
    }
}

