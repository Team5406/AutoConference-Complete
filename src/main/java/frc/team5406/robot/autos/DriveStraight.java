package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
        
        //Create a constraint for the robot. S, V, and A Volts are determined using SysID, your values may vary. 
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.S_VOLTS,
                                       Constants.V_VOLTS,
                                       Constants.A_VOLTS),
            Constants.DRIVE_KINEMATICS,
            10);

    //Set a trajectory Config to be used when following the path set. Set the max speed (Meters per second), and Velocity (Meters per second^2)
    TrajectoryConfig config =
            new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                                 Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.DRIVE_KINEMATICS)
                .addConstraint(autoVoltageConstraint);
                config.setReversed(true);

    //Set the path to be followed - in this case, forward 1.5 meters.
    Trajectory pathOne = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(

        ),
        new Pose2d(1.5, 0, new Rotation2d(Units.degreesToRadians(-90))),
        config
    );

   
        //Create a new RamseteCommand to follow the path
        RamseteCommand ramseteCommand1 = new RamseteCommand(
            pathOne,
            drive::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            Constants.DRIVE_KINEMATICS,
            drive::outputSpeeds,
            drive);

        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            ramseteCommand1.andThen(() -> drive.tankDriveVolts(0, 0))        
        );
      }
    
}