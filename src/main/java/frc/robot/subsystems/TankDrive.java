// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.GyroConstants;

public class TankDrive extends SubsystemBase {
  /** Creates a new TankDrive. */
  CANSparkMax left1;

  CANSparkMax left2;
  CANSparkMax right1;
  CANSparkMax right2;
  DifferentialDrive drivetrain;
  double position = 0;
  // todo
  private final double POSITION_CONVERSION_FACTOR = 1.7391;
  private final double VELOCITY_CONVERSION_FACTOR = 0.001388889;
  private WPI_Pigeon2 gyro = new WPI_Pigeon2(GyroConstants.pigeonID);
  private boolean enableBreaks = true;

  public TankDrive() {

    left1 = new CANSparkMax(DriveTrainConstants.LEFT_1, MotorType.kBrushless);
    left2 = new CANSparkMax(DriveTrainConstants.LEFT_2, MotorType.kBrushless);

    right1 = new CANSparkMax(DriveTrainConstants.RIGHT_1, MotorType.kBrushless);
    right2 = new CANSparkMax(DriveTrainConstants.RIGHT_2, MotorType.kBrushless);
    left1.restoreFactoryDefaults();
    left2.restoreFactoryDefaults();
    right1.restoreFactoryDefaults();
    right2.restoreFactoryDefaults();
    left1.setInverted(true);
    left2.setInverted(true);
    left2.follow(left1);
    right2.follow(right1);

    drivetrain = new DifferentialDrive(left1, right1);
    enableBreaks();

    left1.getEncoder().setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    left2.getEncoder().setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    right1.getEncoder().setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    right2.getEncoder().setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    left1.getEncoder().setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
    left2.getEncoder().setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
    right1.getEncoder().setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
    right2.getEncoder().setPositionConversionFactor(POSITION_CONVERSION_FACTOR);

    left1.getEncoder().setPosition(0);
    left2.getEncoder().setPosition(0);
    right1.getEncoder().setPosition(0);
    right2.getEncoder().setPosition(0);

    gyro.reset();
  }

  public void toggleBreaks() {
    enableBreaks = !enableBreaks;

    if (enableBreaks) {
      this.enableBreaks();
    } else {
      this.disabledbreaks();
    }
  }

  public void setDriveVoltage(double leftvoltage, double rightvoltage) {
    left1.setVoltage(leftvoltage);
    right1.setVoltage(rightvoltage);
  }

  public void drive(double xspeed, double zrotation) {
    
    drivetrain.curvatureDrive(xspeed, zrotation, true);
  }

  public void autoDrive(double xspeed, double zrotation) {

    drivetrain.arcadeDrive(xspeed, zrotation);
  }

  public double getAvergeFPS() {
    return left1.getEncoder().getVelocity() + right1.getEncoder().getVelocity() / 2;
  }

  public double getRightVelocity() {
    return right1.getEncoder().getVelocity();
  }

  public double getLeftVelocity() {
    return left1.getEncoder().getVelocity();
  }

  public double getRobotPosition() {
    return (position);
  }

  public boolean isBalanced() {
    // if within 2.5 deg
    return (Math.abs(gyro.getPitch()) < Constants.GyroConstants.balanceRange);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    position = left1.getEncoder().getPosition();
    SmartDashboard.putNumber("robotposition", getRobotPosition());

    SmartDashboard.putNumber("left1", left1.getEncoder().getPosition());
    SmartDashboard.putNumber("right1", right1.getEncoder().getPosition());

    SmartDashboard.putNumber("Robot Tilt", (int) gyro.getPitch());
    SmartDashboard.putBoolean("Is Balanced", isBalanced());
    SmartDashboard.putData("Gyro", gyro);
    // System.out.println("Roll " + gyro.getRoll() + ", Pitch " + gyro.getPitch() + ", Yaw " +
    // gyro.getYaw());

    SmartDashboard.putNumber("Robot FPS", getAvergeFPS());
  }

  public void resetPosition() {
    position = 0;
    left1.getEncoder().setPosition(0);
    left2.getEncoder().setPosition(0);
    right1.getEncoder().setPosition(0);
    right2.getEncoder().setPosition(0);
  }

  public void enableBreaks() {

    left1.setIdleMode(IdleMode.kBrake);
    left2.setIdleMode(IdleMode.kBrake);
    right1.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kBrake);
  }

  public void disabledbreaks() {
    left1.setIdleMode(IdleMode.kCoast);
    left2.setIdleMode(IdleMode.kCoast);
    right1.setIdleMode(IdleMode.kCoast);
    right2.setIdleMode(IdleMode.kCoast);
  }

  public WPI_Pigeon2 getGyro() {
    return gyro;
  }

  public RobotPosition getStartPosition() {
    return new RobotPosition(
        left1.getEncoder().getPosition(), right1.getEncoder().getPosition(), gyro.getYaw());
  }

  public RobotPosition getRelativePosition(RobotPosition sPosition) {
    return new RobotPosition(
        left1.getEncoder().getPosition() - sPosition.leftposition,
        right1.getEncoder().getPosition() - sPosition.rightposition,
        gyro.getYaw() - sPosition.angle);
  }

  public class RobotPosition {
    public double leftposition;
    public double rightposition;
    public double angle;
    public double averagePosition;

    public RobotPosition(double l, double r, double a) {
      leftposition = l;
      rightposition = r;
      angle = a;
      averagePosition = (l + r) / 2;
    }
  }
}
