package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Spindexer;
import frc.robot.Constants.ShooterConstants.*;
//import frc.robot.subsystems.drive.Drive;
//import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.lib.math.AllianceFlipUtil;
import edu.wpi.first.math.geometry.*;

import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.Spindexer.SPINDEXER_ID;

import javax.print.attribute.standard.PrinterURI;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.controls.Follower;
import frc.robot.subsystems.shooter.ShooterIOKraken;


public class Shooter extends SubsystemBase {
	//private final ShooterIO io;

//   private final TalonFX shooter;
//   private final TalonFX deepFeed;
//   private final TalonFX feeder;
    private final ShooterIO io;

	public Shooter(ShooterIO io) {

//        shooter = new TalonFX(30);
//        feeder = new TalonFX(31);
//        deepFeed = new TalonFX(32);
//
//        bottomShooter = new TalonFX(BOTTOM_SHOOTER_ID);
//        feeder = new TalonFX(FEEDER_ID);

        this.io = io;



//        bottomShooter.setControl(new Follower(topShooter.getDeviceID(), MotorAlignmentValue.Opposed)); // May need to be Inverted

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
//     public double getVelocity () {
//        //pose is just where the robot is at the time
//        var pose = drive.getPose();
//        //First part of the equation, calculating distance
//        double distance = Math.sqrt(
//            Math.pow((Constants.ShooterConstants.HUB_X - pose.getX()), 2) + Math.pow((Constants.ShooterConstants.HUB_Y - pose.getY()), 2)
//     );
//        double velocity;
//
//        //Second part of the equation, converting distance to required velocty
//        velocity = Math.sqrt(
//            9.81 * Math.pow(distance, 2)/ distance * Math.sin(Constants.ShooterConstants.SHOOTER_ANGLE * 2) -
//            Constants.ShooterConstants.BASIN_H * (1+ Math.cos(Constants.ShooterConstants.SHOOTER_ANGLE * 2))
//        );
//        return velocity;
//    }
 
    //Convert velocity to Rotations per second, because talon uses that for some reason
//     public double getRPS () {
//        double RPS;
//        RPS = (getVelocity())/
//            (Math.PI * Constants.ShooterConstants.WHEEL_DIAMETER);
//        return RPS;
//    }
    

//    public VelocityVoltage getVoltage() {
//        VelocityVoltage voltage = new VelocityVoltage(getRPS());
//        return voltage;
//    }

//   public Command aimAndShoot() {
//       return runOnce(() -> shooter.setControl(getVoltage()))
//               .andThen(Commands.waitSeconds(1))
//               .andThen(() -> feeder.set(1))
//               .andThen(Commands.waitSeconds(5))
//               .andThen(() -> feeder.set(1))
//               .andThen(Commands.waitSeconds(1))//TODO set values to run
//               .finallyDo(
//                   () -> {
//                       shooter.set(0);
//                       feeder.set(0);
//                   }
//               );
//   }



    public Command shootTest() {
        return runOnce(() -> io.setShooterSpeed(0.7))
                .andThen(Commands.waitSeconds(1.5))
                .andThen(() -> io.setFeederSpeed(-0.6))
                .andThen(Commands.waitSeconds(30))
                .finallyDo(() -> {
                    io.setShooterSpeed(0);
                    io.setFeederSpeed(0);
                });
    }

    public Command teamFeed() {
        return runOnce(() -> io.setShooterSpeed(0.5))
                .andThen(Commands.waitSeconds(1.5))
                .andThen(() -> io.setFeederSpeed(-0.45))
                .andThen(Commands.waitSeconds(30))
                .finallyDo(() -> {
                    io.setShooterSpeed(0);
                    io.setFeederSpeed(0);
                });
    }

    public Command deepFeed() {
        return runOnce(() -> io.setShooterSpeed(0.95))
                .andThen(Commands.waitSeconds(1.5))
                .andThen(() -> io.setFeederSpeed(-0.6))
                .andThen(Commands.waitSeconds(30))
                .finallyDo(() -> {
                    io.setShooterSpeed(0);
                    io.setFeederSpeed(0);
                });
    }

    public Command newShoot() {
        return runOnce(() -> io.setShooterSpeed(1))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(() -> io.setFeederSpeed(1))
                .andThen((Commands.waitSeconds(30)))
                .finallyDo(() -> {
                    io.setShooterSpeed(0);
                    io.setFeederSpeed(0);
                });
    }

    //FOR TESTING ONLY. PROBOLY DOES NOT WORK!
    public Command oneShoot() {
        return runOnce(
        () -> io.setRightSpeed(1))
        .andThen(Commands.waitSeconds(1.2))
        .andThen(() -> io.setFeederSpeed(1))
        .andThen(Commands.waitSeconds(30))
        .finallyDo(() -> {
            io.setShooterSpeed(0);
            io.setFeederSpeed(0);
        });
    }
}