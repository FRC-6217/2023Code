// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.GyroConstants;

public class PIDDriveTrain extends SubsystemBase {

  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(GyroConstants.pigeonID);

  private final CANSparkMax leftLeader = new CANSparkMax(DriveTrainConstants.LEFT_1, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DriveTrainConstants.LEFT_2, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(DriveTrainConstants.RIGHT_1, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DriveTrainConstants.RIGHT_1, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder(); 

  private final PIDController leftPID = new PIDController(1, 0, 0); //todo add default
  private final PIDController rightPID = new PIDController(1, 0, 0);


  private final double distanceBetweenWheels = 24.5 * 0.0254; //inches to meters
  private final double POSITION_CONVERSION_FACTOR = 1.7391;
  private final double VELOCITY_CONVERSION_FACTOR  = 0.001388889;

  private final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(distanceBetweenWheels);
  private DifferentialDriveOdometry driveOdometry;


  private double driveTrainKS = 0, driveTrainKV = 0; // todo

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(driveTrainKS, driveTrainKV);


  public PIDDriveTrain() {

    leftLeader.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();
    resetEncoders();
    gyro.reset();

    rightLeader.setInverted(true);
    rightFollower.setInverted(true);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightFollower);
    


    leftEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    rightEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

    leftEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
    rightEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);


    driveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());


  }


  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void drive(double xSpeed, double rotation) {
    var speed = driveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rotation));
    setSpeeds(speed);
  }


  private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFF = ff.calculate(speeds.leftMetersPerSecond);
    final double rightFF = ff.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = leftPID.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput = rightPID.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);

    leftLeader.setVoltage( leftFF + leftOutput);
    rightLeader.setVoltage(rightFF + rightOutput);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveOdometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }
}
