package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;

import java.util.List;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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



public class SCurve {

    private final DriveSubsystem drive;
    XboxController driverGamepad = new XboxController(1);

    public SCurve (DriveSubsystem subsystem) {
        drive = subsystem;
      }

    public Command getAutonomousCommand() {
        drive.setHeading();
        drive.reset();
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.AutoConstants.S_VOLTS,
                                       Constants.AutoConstants.V_VOLTS,
                                       Constants.AutoConstants.A_VOLTS),
            Constants.AutoConstants.DRIVE_KINEMATICS,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.AutoConstants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);
            config.setReversed(false);

        /* Rotation2d endpoint = new Rotation2d();
        endpoint = Rotation2d.fromDegrees(45); */

    Trajectory pathOne = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(1, 1), 
                new Translation2d(2, -1)
            ),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        config
    );

    

    
    
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