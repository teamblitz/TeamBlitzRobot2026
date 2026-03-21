package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.lib.util.LimelightHelpers;

import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3;

public class Vision extends SubsystemBase {
    public static final Limelight3 limelight; // Creates the Limelight3
    public static SwerveDrivePoseEstimator poseEstimator;
    private static CommandSwerveDrivetrain drive;

@Override
public void init() {
        
    limelight = hardwareMap.get(Limelight3.class, "limelight"); //initializes the Limelight3
    limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
    limelight.start(); // This tells Limelight to start
}

    @Override
    public void periodic() {
        super.periodic();
        // Runs the the getVisionPose periodicly when the robot is enabled 
        getVisionPose();
    }


    public void getVisionPose() {
        // Gets the robots yaw baised on the Pigeon2
        double robotYaw = drive.getPigeon2().getYaw().getValueAsDouble();
        // Sets the robots orintation values to 0 when in starting postion
        LimelightHelpers.SetRobotOrientation("limelight", robotYaw, 0, 0, 0, 0, 0);

        //Imports tag data
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

