package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.*;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.lib.math.AllianceFlipUtil;
import edu.wpi.first.math.geometry.*;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase {
	//private final ShooterIO io;

    private final TalonFX shooter;
    private final TalonFX feeder;
    private final TalonFX deepFeed;
    private final CommandSwerveDrivetrain drive;

	public Shooter(CommandSwerveDrivetrain drive) {
	
        shooter = new TalonFX(30);
        feeder = new TalonFX(1);
        deepFeed = new TalonFX(32);
        this.drive = drive;
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
    public double getVelocity () {
        //pose is just where the robot is at the time
        var pose = drive.getState().Pose;
        //First part of the equation, calculating distance
        double distance = Math.sqrt(
            Math.pow((Constants.Shooter.HUB_X - pose.getX()), 2) + Math.pow((Constants.Shooter.HUB_Y - pose.getY()), 2)
        );
        double velocity;

        //Second part of the equation, converting distance to required velocty
        velocity = Math.sqrt(
            9.81 * Math.pow(distance, 2)/ distance * Math.sin(Constants.Shooter.SHOOTER_ANGLE * 2) - 
            Constants.Shooter.BASIN_H * (1+ Math.cos(Constants.Shooter.SHOOTER_ANGLE * 2))
        );
        return velocity;
    }

    //Convert velocity to Rotations per second, because talon uses that for some reason
    public double getRPS () {
        double RPS;
        RPS = (getVelocity())/
            (Math.PI * Constants.Shooter.WHEEL_DIAMETER);
        return RPS;
    }

    public VelocityVoltage getVoltage() {
        VelocityVoltage voltage = new VelocityVoltage(getRPS());
        return voltage;
    }

    public Command aimAndShoot() {
        return runOnce(() -> shooter.setControl(getVoltage()))
                .andThen(Commands.waitSeconds(1))
                .andThen(() -> deepFeed.set(1))
                .andThen(Commands.waitSeconds(5))
                .andThen(() -> deepFeed.set(1))
                .andThen(Commands.waitSeconds(1))//TODO set values to run
                .finallyDo(
                    () -> {
                        shooter.set(0);
                        deepFeed.set(0);
                    }
                );
    }



    public Command shootTest() {
        return runOnce(() -> shooter.set(-1))
                .andThen(Commands.waitSeconds(1))
                .andThen(() -> deepFeed.set(1))
                .andThen(Commands.waitSeconds(5))
                .andThen(() -> deepFeed.set(1))
                .andThen(Commands.waitSeconds(1))//TODO set values to run
                .finallyDo(
                    () -> {
                        shooter.set(0);
                        deepFeed.set(0);
                    }
                );

    }
}
