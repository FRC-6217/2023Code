// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.GyroConstants;

public class PIDDriveTrain extends SubsystemBase {

  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(GyroConstants.pigeonID);

  private final CANSparkMax leftLeader =
      new CANSparkMax(DriveTrainConstants.LEFT_1, MotorType.kBrushless);
  private final CANSparkMax leftFollower =
      new CANSparkMax(DriveTrainConstants.LEFT_2, MotorType.kBrushless);
  private final CANSparkMax rightLeader =
      new CANSparkMax(DriveTrainConstants.RIGHT_1, MotorType.kBrushless);
  private final CANSparkMax rightFollower =
      new CANSparkMax(DriveTrainConstants.RIGHT_2, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  private final PIDController leftPID = new PIDController(.01, .0010, .0010); // todo add default
  private final PIDController rightPID = new PIDController(.01, .0010, .0010);

  private final double distanceBetweenWheels = Units.inchesToMeters(Constants.uniqueRobotConstants.getDistanceBetweenWheelsInches());
  private final double POSITION_CONVERSION_FACTOR = 1.7391;
  private final double VELOCITY_CONVERSION_FACTOR = 0.001388889;
  private double maxSpeed = 11.5; //fps
  private double maxRotationSpeed = 2 * Math.PI; //one rev per second
  private boolean enableBreaks = true;

  private final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(distanceBetweenWheels);
  private DifferentialDriveOdometry driveOdometry;
  public DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds();
  private Pose2d pose = new Pose2d();

  private double driveTrainLeftKS = Constants.uniqueRobotConstants.getLeftDriveTrainMotorConstants().ks;
  private double driveTrainLeftKV = Constants.uniqueRobotConstants.getLeftDriveTrainMotorConstants().kv;
  private double driveTrainRightKS = Constants.uniqueRobotConstants.getRightDriveTrainMotorConstants().ks;
  private double driveTrainRightKV = Constants.uniqueRobotConstants.getRightDriveTrainMotorConstants().kv;

  private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(driveTrainLeftKS, driveTrainLeftKV);
  private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(driveTrainRightKS, driveTrainRightKV);

  private double lFF = 0;
  private double rFF = 0;
  private double rightVoltage = 0;
  private double leftVoltage = 0;
  private double leftPIDOutput = 0;
  private double rightPIDOutput = 0;

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
    
    enableBreaks();


    driveOdometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putData(this);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void drive(double xSpeed, double rotation) {
    speeds = driveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed * Units.feetToMeters(maxSpeed), 0, rotation * maxRotationSpeed));
    setSpeeds(speeds);
  }

  private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    lFF = leftFF.calculate(speeds.leftMetersPerSecond);
    rFF = rightFF.calculate(speeds.rightMetersPerSecond);

          ///@TODO what units is getVelocity?
    leftPIDOutput = leftPID.calculate(Units.feetToMeters(leftEncoder.getVelocity()), speeds.leftMetersPerSecond);
    rightPIDOutput = rightPID.calculate(Units.feetToMeters(rightEncoder.getVelocity()), speeds.rightMetersPerSecond);

    leftVoltage = lFF + leftPIDOutput;
    rightVoltage = rFF + rightPIDOutput;
    leftLeader.setVoltage(leftVoltage);
    rightLeader.setVoltage(rightVoltage);
  }

  public void toggleBreaks() {
    enableBreaks = !enableBreaks;

    if (enableBreaks) {
      this.enableBreaks();
    } else {
      this.disableBreaks();
    }
  }

  public void enableBreaks() {
    enableBreaks = true;
    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
  }

  public void disableBreaks() {
    enableBreaks = false;
    leftLeader.setIdleMode(IdleMode.kCoast);
    leftFollower.setIdleMode(IdleMode.kCoast);
    rightLeader.setIdleMode(IdleMode.kCoast);
    rightFollower.setIdleMode(IdleMode.kCoast);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = driveOdometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }


  //*************SmartDashboard **************/

  public void setLeftPID(double[] in) {
    leftPID.setP(in[0]);
    leftPID.setI(in[1]);
    leftPID.setD(in[2]);
  }

  public void setRightPID(double[] in) {
    rightPID.setP(in[0]);
    rightPID.setI(in[1]);
    rightPID.setD(in[2]);
  }


  public double[] getLeftPID() {
    return new  double[] {leftPID.getP(), leftPID.getI(), leftPID.getD()};
  }

  public double[] getRightPID() {
    return new  double[] {rightPID.getP(), rightPID.getI(), rightPID.getD()};
  }

  public double[] getPose() {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
  }

  public double[] getFeedForward() {
    return new double[] {lFF, rFF};
  }

  public double[] getVoltages() {
    return new double[] {leftVoltage, rightVoltage};
  }

  public double[] getSetPointSpeeds() {
    return new double[] {Units.metersToFeet(speeds.leftMetersPerSecond), Units.metersToFeet(speeds.rightMetersPerSecond)};
  }

  public double[] getActualVelocity() {
    return new double[] {leftEncoder.getVelocity(), rightEncoder.getVelocity()};
  }

  public double[] getPIDOutput() {
    return new double[] {leftPIDOutput, rightPIDOutput};
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");
    builder.addDoubleArrayProperty(".rightPID", () -> getRightPID(), (in) -> setRightPID(in));
    builder.addDoubleArrayProperty(".lefttPID", () -> getLeftPID(), (in) -> setLeftPID(in));
    builder.addDoubleArrayProperty(".pose", () -> getPose(), null);
    builder.addDoubleArrayProperty(".feedForward", () -> getFeedForward(), null);
    builder.addDoubleArrayProperty(".voltages", () -> getVoltages(), null);
    builder.addBooleanProperty(".breakMode", () -> enableBreaks, null);
    builder.addDoubleArrayProperty(".speedSetpoint", () -> getSetPointSpeeds(), null);
    builder.addDoubleArrayProperty(".actualVelocity", () -> getActualVelocity(), null);
    builder.addDoubleArrayProperty(".PIDOutput", () -> getPIDOutput(), null);

  }
}
