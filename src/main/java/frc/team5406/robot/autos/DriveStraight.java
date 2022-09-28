package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;

import java.util.List;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
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
        drive.resetGyro();
        drive.reset();
        drive.setBrakeMode(true);
        
        //Create a constraint for the robot. S, V, and A Volts are determined using SysID, your values may vary. 
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.AutoConstants.S_VOLTS,
                                       Constants.AutoConstants.V_VOLTS,
                                       Constants.AutoConstants.A_VOLTS),
            Constants.AutoConstants.DRIVE_KINEMATICS,
            10);

    //Set a trajectory Config to be used when following the path set. Set the max speed (Meters per second), and Velocity (Meters per second^2)
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.AutoConstants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);
            config.setReversed(false);

        /* Rotation2d endpoint = new Rotation2d();
        endpoint = Rotation2d.fromDegrees(45); */

    //Set the path to be followed - in this case, forward 1.5 meters.
    Trajectory pathOne = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(

        ),
        new Pose2d(1.6, 0, new Rotation2d(0)),
        config
    );

    

    
        //Create a new RamseteCommand to follow the path
        RamseteCommand ramseteCommand = new RamseteCommand(
          
            pathOne,
            drive::getPose,
            new RamseteController(Constants.AutoConstants.RAMSETE_B, Constants.AutoConstants.RAMSETE_ZETA),
            Constants.AutoConstants.DRIVE_KINEMATICS,
            drive::outputSpeeds,
            drive);

        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0))
        );
      }
    
}