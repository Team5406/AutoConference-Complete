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
  private CANSparkMax leftDrive = new CANSparkMax(Constants.MotorPorts.MOTOR_DRIVE_LEFT_ONE, MotorType.kBrushless);
  private CANSparkMax leftDriveFollower = new CANSparkMax(Constants.MotorPorts.MOTOR_DRIVE_LEFT_TWO,
      MotorType.kBrushless);
  private CANSparkMax rightDrive = new CANSparkMax(Constants.MotorPorts.MOTOR_DRIVE_RIGHT_ONE, MotorType.kBrushless);
  private CANSparkMax rightDriveFollower = new CANSparkMax(Constants.MotorPorts.MOTOR_DRIVE_RIGHT_TWO,
      MotorType.kBrushless);
  private RelativeEncoder leftEncoder, rightEncoder;
  private SparkMaxPIDController leftMotorPID, rightMotorPID;

  private DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);
  SimpleMotorFeedforward driveTrain = new SimpleMotorFeedforward(Constants.AutoConstants.S_VOLTS,
      Constants.AutoConstants.V_VOLTS,
      Constants.AutoConstants.A_VOLTS);

  AHRS gyro = new AHRS(SPI.Port.kMXP);
  Pose2d pose = new Pose2d();
  DifferentialDriveOdometry odometry;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Constants.AutoConstants.TRACK_WIDTH_METERS);

  public void setupMotors() {
    leftDriveFollower.restoreFactoryDefaults();
    rightDriveFollower.restoreFactoryDefaults();
    rightDrive.restoreFactoryDefaults();
    leftDrive.restoreFactoryDefaults();

    leftDriveFollower.follow(leftDrive, false);
    rightDriveFollower.follow(rightDrive, false);
    rightDrive.setInverted(false);
    leftDrive.setInverted(true);
    drive.setSafetyEnabled(false);

    leftDrive.setSmartCurrentLimit(Constants.CurrentLimit.CURRENT_LIMIT_DRIVE_LEFT);
    leftDriveFollower.setSmartCurrentLimit(Constants.CurrentLimit.CURRENT_LIMIT_DRIVE_LEFT);
    rightDrive.setSmartCurrentLimit(Constants.CurrentLimit.CURRENT_LIMIT_DRIVE_RIGHT);
    rightDriveFollower.setSmartCurrentLimit(Constants.CurrentLimit.CURRENT_LIMIT_DRIVE_RIGHT);
    leftMotorPID = leftDrive.getPIDController();
    rightMotorPID = rightDrive.getPIDController();
    leftEncoder = leftDrive.getEncoder();
    rightEncoder = rightDrive.getEncoder();

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

    rightDrive.setOpenLoopRampRate(0.05);
    leftDrive.setOpenLoopRampRate(0.05);

    leftEncoder.setPositionConversionFactor(
        Constants.GearRatios.GEAR_RATIO_DRIVE * Math.PI * Constants.Other.DRIVE_WHEEL_DIAMETER);
    leftEncoder.setVelocityConversionFactor(
        (Constants.GearRatios.GEAR_RATIO_DRIVE * Math.PI * Constants.Other.DRIVE_WHEEL_DIAMETER) / 60);

    rightEncoder.setPositionConversionFactor(
        Constants.GearRatios.GEAR_RATIO_DRIVE * Math.PI * Constants.Other.DRIVE_WHEEL_DIAMETER);
    rightEncoder.setVelocityConversionFactor(
        (Constants.GearRatios.GEAR_RATIO_DRIVE * Math.PI * Constants.Other.DRIVE_WHEEL_DIAMETER) / 60);

    resetEncoders();

    leftDrive.burnFlash();
    leftDriveFollower.burnFlash();
    rightDrive.burnFlash();
    rightDriveFollower.burnFlash();
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  public void setBrakeMode(boolean brake) {
    IdleMode brakeMode = (brake ? IdleMode.kBrake : IdleMode.kCoast);
    leftDrive.setIdleMode(brakeMode);
    leftDriveFollower.setIdleMode(brakeMode);
    rightDrive.setIdleMode(brakeMode);
    rightDriveFollower.setIdleMode(brakeMode);
  }

  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }

  public double getRightSpeed() {
    return rightEncoder.getVelocity();
  }

  public double getAverageSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  public double getLeftDistance() {
    return leftEncoder.getPosition();
  }

  public double getRightDistance() {
    return rightEncoder.getPosition();
  }

  // Reset Encoders
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle() * (Constants.Other.GYRO_REVERSED ? -1.0 : 1.0));
  }

  public void setHeading() {
    gyro.zeroYaw();
  }

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

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
    drive.feed();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void reset() {
    setHeading();
    resetEncoders();
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  public void setPose(Pose2d pose) {
    resetEncoders();
  }

  public DriveSubsystem() {
    setupMotors();
  }

  @Override
  public void periodic() {
  }

}
