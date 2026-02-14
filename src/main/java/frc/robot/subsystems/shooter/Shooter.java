package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.*;
import frc.lib.math.AllianceFlipUtil;
import edu.wpi.first.math.geometry.*;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import static frc.robot.Constants.Shooter.DEEP_FEEDER_ID;
import static frc.robot.Constants.Shooter.FEEDER_ID;
import static frc.robot.Constants.Shooter.SHOOTER_ID;

import com.ctre.phoenix6.controls.Follower;


public class Shooter extends SubsystemBase {
	//private final ShooterIO io;

    private final TalonFX shooter;
    private final TalonFX feeder;
    private final TalonFX deepFeed;
    //private final CommandSwerveDrivetrain drive;

	public Shooter() {
	
        shooter = new TalonFX(SHOOTER_ID);

        feeder = new TalonFX(FEEDER_ID);
        deepFeed = new TalonFX(DEEP_FEEDER_ID);
        //this.drive = drive;

        feeder.setControl(new Follower(shooter.getDeviceID(), MotorAlignmentValue.Opposed)); // May need to be Inverted
	}

	@Override
	public void periodic() {
		super.periodic();
		// IO update would be done by a higher-level manager; keep minimal here
	}

    
    /** This method calculates the required velocity for the ball,
     * given the distance to target.
     * 
     *
     * @return velocity, the required velocity
     * 
     */
    // public double getVelocity () {
    //     var pose = drive.getState().Pose;
    //     double distance = Math.sqrt(
    //         Math.pow((Constants.Shooter.HUB_X - pose.getX()), 2) + Math.pow((Constants.Shooter.HUB_Y - pose.getY()), 2)
    //     );
    //     double velocity;

    //     velocity = Math.sqrt(
    //         9.81 * Math.pow(distance, 2)/ distance * Math.sin(Constants.Shooter.SHOOTER_ANGLE * 2) - 
    //         Constants.Shooter.BASIN_H * (1+ Math.cos(Constants.Shooter.SHOOTER_ANGLE * 2))
    //     );
    //     return velocity;
    // }

    // public double getRPM () {
    //     double RPM;
    //     RPM = (getVelocity() * 60)/
    //         (Math.PI * Constants.Shooter.WHEEL_DIAMETER);
    //     return RPM;
    // }

    // public double getVoltage() {
    //     double voltage;
    //     voltage = new VelocityVoltage(getRPM() / 60);
    //     return voltage;
    // }



    public Command shootTest() {
        return runOnce(() -> shooter.set(-1))
                .andThen(Commands.waitSeconds(1))
                .andThen(() -> deepFeed.set(0.6))
                .andThen(Commands.waitSeconds(5))
                .andThen(() -> deepFeed.set(0.6))
                .andThen(Commands.waitSeconds(1))//TODO set values to run
                .finallyDo(
                    () -> {
                        shooter.set(0);
                        deepFeed.set(0);
                    }
                );

    }
}
