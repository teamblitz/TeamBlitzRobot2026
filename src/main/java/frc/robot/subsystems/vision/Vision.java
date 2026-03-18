package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.lib.util.LimelightHelpers;



public class Vision extends SubsystemBase {
    public static final Limelight3 limelight;
    public static SwerveDrivePoseEstimator poseEstimator;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }   

    @Override
    public void periodic() {
        super.periodic();

        getVisionPose();
    }


    public void getVisionPose() {
        double robotYaw = gyro.getYaw();
        LimelightHelpers.SetRobotOrientation("limelight", robotYaw, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate limelightMeasurement = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds
        );
    }
}

