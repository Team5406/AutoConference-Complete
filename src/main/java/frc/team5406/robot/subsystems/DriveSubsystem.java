// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  //Create 4 new SparkMAX Objects - Used to control the drive motors. 
  private CANSparkMax leftDrive = new CANSparkMax(Constants.MotorPorts.MOTOR_DRIVE_LEFT_ONE, MotorType.kBrushless);
  private CANSparkMax leftDriveFollower = new CANSparkMax(Constants.MotorPorts.MOTOR_DRIVE_LEFT_TWO,
      MotorType.kBrushless);
  private CANSparkMax rightDrive = new CANSparkMax(Constants.MotorPorts.MOTOR_DRIVE_RIGHT_ONE, MotorType.kBrushless);
  private CANSparkMax rightDriveFollower = new CANSparkMax(Constants.MotorPorts.MOTOR_DRIVE_RIGHT_TWO,
      MotorType.kBrushless);

  //Create 2 Encoder objects - used to read the Position and Velocity values from the motors.
  private RelativeEncoder leftEncoder, rightEncoder;

  //Create 2 PID Controllers
  private SparkMaxPIDController leftMotorPID, rightMotorPID;

  private DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);
  SimpleMotorFeedforward driveTrain = new SimpleMotorFeedforward(Constants.AutoConstants.S_VOLTS,
      Constants.AutoConstants.V_VOLTS,
      Constants.AutoConstants.A_VOLTS);

  //Create a new NavX Object - used for Gyro and Odoemtry
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  Pose2d pose = new Pose2d();
  DifferentialDriveOdometry odometry;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Constants.AutoConstants.TRACK_WIDTH_METERS);


  /**
   * Setup the motors each time a "DriveSubsystem" object is made.
   */
  public void setupMotors() {
    //Restore each of the SparkMax Controllers to Factory Settings.
    leftDriveFollower.restoreFactoryDefaults();
    rightDriveFollower.restoreFactoryDefaults();
    rightDrive.restoreFactoryDefaults();
    leftDrive.restoreFactoryDefaults();

    //Have the "follower" motors follow the main SparkMAX's
    leftDriveFollower.follow(leftDrive, false);
    rightDriveFollower.follow(rightDrive, false);

    //Invert the left side of the drive train.
    rightDrive.setInverted(false);
    leftDrive.setInverted(true);

    //Disable motor safety for our drivetrain - Since we are not using PWM, it doesn't make too much of a difference
    drive.setSafetyEnabled(false);

    //Set each SparkMAX to a current limit of 40amps.
    leftDrive.setSmartCurrentLimit(Constants.CurrentLimit.CURRENT_LIMIT_DRIVE_LEFT);
    leftDriveFollower.setSmartCurrentLimit(Constants.CurrentLimit.CURRENT_LIMIT_DRIVE_LEFT);
    rightDrive.setSmartCurrentLimit(Constants.CurrentLimit.CURRENT_LIMIT_DRIVE_RIGHT);
    rightDriveFollower.setSmartCurrentLimit(Constants.CurrentLimit.CURRENT_LIMIT_DRIVE_RIGHT);

    //Get the PID controllers and encoder profiles from the SparkMAXs
    leftMotorPID = leftDrive.getPIDController();
    rightMotorPID = rightDrive.getPIDController();
    leftEncoder = leftDrive.getEncoder();
    rightEncoder = rightDrive.getEncoder();

    //Set PID Values for each Motor.
    leftMotorPID.setP(Constants.PID.LEFT_DRIVE_PID0_P, 0);
    leftMotorPID.setI(Constants.PID.LEFT_DRIVE_PID0_I, 0);
    leftMotorPID.setD(Constants.PID.LEFT_DRIVE_PID0_D, 0);
    leftMotorPID.setIZone(0, 0);
    leftMotorPID.setFF(Constants.PID.LEFT_DRIVE_PID0_F, 0);
    leftMotorPID.setOutputRange(Constants.PID.OUTPUT_RANGE_MIN, Constants.PID.OUTPUT_RANGE_MAX, 0);

    leftMotorPID.setP(Constants.PID.LEFT_DRIVE_PID1_P, 1);
    leftMotorPID.setI(Constants.PID.LEFT_DRIVE_PID1_I, 1);
    leftMotorPID.setD(Constants.PID.LEFT_DRIVE_PID1_D, 1);
    leftMotorPID.setIZone(0, 1);
    leftMotorPID.setFF(Constants.PID.LEFT_DRIVE_PID1_F, 1);
    leftMotorPID.setOutputRange(Constants.PID.OUTPUT_RANGE_MIN, Constants.PID.OUTPUT_RANGE_MAX, 1);

    rightMotorPID.setP(Constants.PID.RIGHT_DRIVE_PID0_P, 0);
    rightMotorPID.setI(Constants.PID.RIGHT_DRIVE_PID0_I, 0);
    rightMotorPID.setD(Constants.PID.RIGHT_DRIVE_PID0_D, 0);
    rightMotorPID.setIZone(0, 0);
    rightMotorPID.setFF(Constants.PID.RIGHT_DRIVE_PID0_F, 0);
    rightMotorPID.setOutputRange(Constants.PID.OUTPUT_RANGE_MIN, Constants.PID.OUTPUT_RANGE_MAX, 0);

    rightMotorPID.setP(Constants.PID.RIGHT_DRIVE_PID1_P, 1);
    rightMotorPID.setI(Constants.PID.RIGHT_DRIVE_PID1_I, 1);
    rightMotorPID.setD(Constants.PID.RIGHT_DRIVE_PID1_D, 1);
    rightMotorPID.setIZone(0, 1);
    rightMotorPID.setFF(Constants.PID.RIGHT_DRIVE_PID1_F, 1);
    rightMotorPID.setOutputRange(Constants.PID.OUTPUT_RANGE_MIN, Constants.PID.OUTPUT_RANGE_MAX, 1);

    //Set the time it takes to go from 0 to full power on the drive motors.
    rightDrive.setOpenLoopRampRate(0.05);
    leftDrive.setOpenLoopRampRate(0.05);

    //Set the Conversion factors of the encoders from the default units to Meters and Meters/Second
    leftEncoder.setPositionConversionFactor(
        Constants.GearRatios.GEAR_RATIO_DRIVE * Math.PI * Constants.Other.DRIVE_WHEEL_DIAMETER);
    leftEncoder.setVelocityConversionFactor(
        (Constants.GearRatios.GEAR_RATIO_DRIVE * Math.PI * Constants.Other.DRIVE_WHEEL_DIAMETER) / 60);

    rightEncoder.setPositionConversionFactor(
        Constants.GearRatios.GEAR_RATIO_DRIVE * Math.PI * Constants.Other.DRIVE_WHEEL_DIAMETER);
    rightEncoder.setVelocityConversionFactor(
        (Constants.GearRatios.GEAR_RATIO_DRIVE * Math.PI * Constants.Other.DRIVE_WHEEL_DIAMETER) / 60);
    
    
    resetEncoders();

    //Burn all the changes to the SparkMAX's flash.
    leftDrive.burnFlash();
    leftDriveFollower.burnFlash();
    rightDrive.burnFlash();
    rightDriveFollower.burnFlash();
  }

  /**
   * 
   * @param speed - Speed you'd wish to go
   * @param rotation - Speed you'd wish to turn
   */
  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  /**
   * 
   * @param break - Boolean, true break, false coast.
   */
  public void setBrakeMode(boolean brake) {
    IdleMode brakeMode = (brake ? IdleMode.kBrake : IdleMode.kCoast);
    leftDrive.setIdleMode(brakeMode);
    leftDriveFollower.setIdleMode(brakeMode);
    rightDrive.setIdleMode(brakeMode);
    rightDriveFollower.setIdleMode(brakeMode);
  }

  /**
   *
   * @return Left Encoders velocity (M/S)
   */
  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }

  /**
   *
   * @return Right Encoders velocity (M/S)
   */
  public double getRightSpeed() {
    return rightEncoder.getVelocity();
  }

    /**
   *
   * @return Encoders Average velocity (M/S)
   */
  public double getAverageSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  /**
   * 
   * @return Left Distance travelled since start
   */
  public double getLeftDistance() {
    return leftEncoder.getPosition();
  }

  /**
   * 
   * @return Right Distance travelled since start
   */
  public double getRightDistance() {
    return rightEncoder.getPosition();
  }

  // Reset Encoders
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * 
   * @return Rotation turned from the Gyro
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle() * (Constants.Other.GYRO_REVERSED ? -1.0 : 1.0));
  }

  /**
   * Reset the Gyro to Zero.
   */
  public void resetGyro() {
    gyro.zeroYaw();
  }

  /**
   * Uses PID control, as well as a FeedFoward Value to determine how fast to move each motor to reach a intented target.
   * @param leftSpeed
   * @param rightSpeed
   */
  public void outputSpeeds(double leftSpeed, double rightSpeed) {
    double origLeftSpeed = leftSpeed;
    double origRightSpeed = rightSpeed;
    leftSpeed /= Units.inchesToMeters(Constants.Other.INCHES_PER_REV / Constants.Other.SECONDS_PER_MINUTE);
    rightSpeed /= Units.inchesToMeters(Constants.Other.INCHES_PER_REV / Constants.Other.SECONDS_PER_MINUTE);
    SmartDashboard.putNumber("Orig Left Speed", origLeftSpeed);
    SmartDashboard.putNumber("Orig Right Speed", origRightSpeed);
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);

    SmartDashboard.putNumber("X Translation", pose.getTranslation().getX());
    SmartDashboard.putNumber("Left Speed (A)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Speed (A)", rightEncoder.getVelocity());

    double arbFFLeft = driveTrain.calculate(origLeftSpeed);
    double arbFFRight = driveTrain.calculate(origRightSpeed);
    SmartDashboard.putNumber("Arb FF L", arbFFLeft);
    SmartDashboard.putNumber("Arb FF R", arbFFRight);

    leftMotorPID.setReference(leftSpeed, ControlType.kVelocity, 0, arbFFLeft,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
    rightMotorPID.setReference(rightSpeed, ControlType.kVelocity, 0, arbFFRight,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
    drive.feed();
  }

  /**
   * @param leftVolts
   * @param rightVolts
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
    drive.feed();
  }

  /**
   * 
   * @return Position travelled in Meters from Gyro.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Reset the gyro and encoders on the start of Auto.
   */
  public void reset() {
    resetGyro();
    resetEncoders();
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  /**
   * Reset the Odemetry
   * @param pose - Position on the field
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  /**
   * Setup all the motors each time this class is created.
   */
  public DriveSubsystem() {
    setupMotors();
  }

  @Override
  public void periodic() {
  }

}
