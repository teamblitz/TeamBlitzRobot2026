package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface FuelVisionIO {

    @AutoLog
    public class FuelVisionInputs {
        public double tx;
        public double ty;

        public double txPixels;
        public double tyPixels;

        public double botSpaceX;
        public double botSpaceY;

        public boolean valid;
        public boolean projectionValid;

        public double timestampCapture;
    }

    public default void updateInputs(FuelVisionIO.FuelVisionInputs inputs) {}
    
}
