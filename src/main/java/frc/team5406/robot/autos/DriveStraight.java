package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import frc.team5406.robot.subsystems.DriveSubsystem;



public class DriveStraight {

    private final DriveSubsystem drive;

    public DriveStraight (DriveSubsystem subsystem) {
        drive = subsystem;
      }

    public Command getAutonomousCommand() {
        //Reset the Gyro and Odoemtry on the robot at the start of Auto.
        drive.reset();
        drive.setBrakeMode(true);

        RamseteController disabledRamsete = new RamseteController() {
            @Override
            public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                    double angularVelocityRefRadiansPerSecond) {
                return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
            }
        };
        
        //Create a constraint for the robot. S, V, and A Volts are determined using SysID, your values may vary. 
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.AutoConstants.S_VOLTS,
                                       Constants.AutoConstants.V_VOLTS,
                                       Constants.AutoConstants.A_VOLTS),
            Constants.AutoConstants.DRIVE_KINEMATICS,
            10);

        var autoCurveConstraint = new CentripetalAccelerationConstraint(0.5);

    //Set a trajectory Config to be used when following the path set. Set the max speed (Meters per second), and Velocity (Meters per second^2)
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.AutoConstants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint)
            .addConstraint(autoCurveConstraint);
            config.setReversed(false);

    TrajectoryConfig config2 =
            new TrajectoryConfig(Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                                 Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.AutoConstants.DRIVE_KINEMATICS)
                .addConstraint(autoVoltageConstraint);
                config2.setReversed(true);
    

        /* Rotation2d endpoint = new Rotation2d();
        endpoint = Rotation2d.fromDegrees(45); */

    //Set the path to be followed - in this case, forward 1.5 meters.
    Trajectory pathOne = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            //new Translation2d(1, -0.5), 
            //new Translation2d(2, 0.5)

        ),
        new Pose2d(1.5, -1.5, new Rotation2d(Units.degreesToRadians(-90))),
        config
    );

    Trajectory pathTwo = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.5, -1.5, new Rotation2d(Units.degreesToRadians(-90))),
        List.of(

        ),
        new Pose2d(3, 0, new Rotation2d(Units.degreesToRadians(-180))),
        config2
    );
    

   
        //Create a new RamseteCommand to follow the path
        RamseteCommand ramseteCommand1 = new RamseteCommand(
          
            pathOne,
            drive::getPose,
            new RamseteController(Constants.AutoConstants.RAMSETE_B, Constants.AutoConstants.RAMSETE_ZETA),
            Constants.AutoConstants.DRIVE_KINEMATICS,
            drive::outputSpeeds,
            drive);

            RamseteCommand ramseteCommand2 = new RamseteCommand(
          
                pathTwo,
                drive::getPose,
                new RamseteController(Constants.AutoConstants.RAMSETE_B, Constants.AutoConstants.RAMSETE_ZETA),
                Constants.AutoConstants.DRIVE_KINEMATICS,
                drive::outputSpeeds,
                drive);

        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            ramseteCommand1.andThen(() -> drive.tankDriveVolts(0, 0)),
            ramseteCommand2.andThen(() -> drive.tankDriveVolts(0, 0))           
        );
      }
    
}