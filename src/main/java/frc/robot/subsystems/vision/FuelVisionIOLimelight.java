package frc.robot.subsystems.vision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import frc.lib.util.LimelightHelpers;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

public class FuelVisionIOLimelight implements FuelVisionIO {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    
}
