package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.lib.math.AllianceFlipUtil;
import edu.wpi.first.math.geometry.*;

import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.ShooterConstants.*;

import javax.print.attribute.standard.PrinterURI;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.controls.Follower;


public class Shooter extends SubsystemBase {
	//private final ShooterIO io;

   private final TalonFX rightShooter;
   private final TalonFX leftShooter;
   private final TalonFX feeder;
    private final ShooterIO io;
    private final Drive drive;

	public Shooter(ShooterIO io, Drive drive) {
        rightShooter = new TalonFX(RIGHT_SHOOTER_ID);
        leftShooter = new TalonFX(LEFT_SHOOTER_ID);
        feeder = new TalonFX(FEEDER_ID);

        this.io = io;
        this.drive= drive;
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
        var pose = drive.getPose();
        //First part of the equation, calculating distance
        double distance = Math.sqrt(
            Math.pow((Constants.ShooterConstants.HUB_X - pose.getX()), 2) + Math.pow((Constants.ShooterConstants.HUB_Y - pose.getY()), 2)
     );
        double velocity;

        //Second part of the equation, converting distance to required velocty
        velocity = Math.sqrt(
            9.81 * Math.pow(distance, 2)/ distance * Math.sin(Constants.ShooterConstants.SHOOTER_ANGLE * 2) -
            Constants.ShooterConstants.BASIN_H * (1+ Math.cos(Constants.ShooterConstants.SHOOTER_ANGLE * 2))
        );
        return velocity;
    }
 
    //Convert velocity to Rotations per second, because talon uses that for some reason
     public double getRPS () {
        double RPS;
        RPS = (getVelocity())/
            (Math.PI * Constants.ShooterConstants.WHEEL_DIAMETER);
        return RPS;
    }
    

    public VelocityVoltage getVoltage() {
        VelocityVoltage voltage = new VelocityVoltage(getRPS());
        return voltage;
    }

   public Command aimAndShoot() {
       return runOnce(() -> rightShooter.setControl(getVoltage()))
               .andThen(Commands.waitSeconds(1))
               .andThen(() -> feeder.set(1))
               .andThen(Commands.waitSeconds(5))
               .andThen(() -> feeder.set(1))
               .andThen(Commands.waitSeconds(1))//TODO set values to run
               .finallyDo(
                   () -> {
                       rightShooter.set(0);
                       feeder.set(0);
                   }
               );
   }

    public Command defaultShoot(double shooterSpeed, double feederSpeed) {
        return runOnce(() -> io.setShooterSpeed(shooterSpeed))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(() -> io.setFeederSpeed(feederSpeed))
                .andThen(Commands.idle())
                .finallyDo(() -> {
                    io.setShooterSpeed(0);
                    io.setFeederSpeed(0);
                });
    }
}
