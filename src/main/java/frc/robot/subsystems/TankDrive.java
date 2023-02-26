// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  RobotPosition position = new RobotPosition(0, 0,0);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry pipeline = table.getEntry("pipeline");

  private LimeData  limeData = new LimeData();

  SlewRateLimiter slewFilter = new SlewRateLimiter(DriveTrainConstants.rampSpeedInSeconds);
  
  private WPI_Pigeon2 gyro = new WPI_Pigeon2(GyroConstants.pigeonID);
  private boolean enableBreaks = false;
  private boolean isTurningEnabled = true;
  private double maxMPS = 0;

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
    //REMOVE THIS AT SOMEPOINT IF YOU FEEL LIKE IT
    drivetrain.setSafetyEnabled(false);
    enableBreaks();

    left1.getEncoder().setVelocityConversionFactor(Constants.uniqueRobotConstants.getDriveTrainVelocityConversion());
    left2.getEncoder().setVelocityConversionFactor(Constants.uniqueRobotConstants.getDriveTrainVelocityConversion());
    right1.getEncoder().setVelocityConversionFactor(Constants.uniqueRobotConstants.getDriveTrainVelocityConversion());
    right2.getEncoder().setVelocityConversionFactor(Constants.uniqueRobotConstants.getDriveTrainVelocityConversion());
    left1.getEncoder().setPositionConversionFactor(Constants.uniqueRobotConstants.getDriveTrainPositionConversion());
    left2.getEncoder().setPositionConversionFactor(Constants.uniqueRobotConstants.getDriveTrainPositionConversion());
    right1.getEncoder().setPositionConversionFactor(Constants.uniqueRobotConstants.getDriveTrainPositionConversion());
    right2.getEncoder().setPositionConversionFactor(Constants.uniqueRobotConstants.getDriveTrainPositionConversion());

    left1.getEncoder().setPosition(0);
    left2.getEncoder().setPosition(0);
    right1.getEncoder().setPosition(0);
    right2.getEncoder().setPosition(0);

    gyro.reset();

  }
  public void toggleTurning(){
    isTurningEnabled = !isTurningEnabled;
  }

  public void toggleBreaks() {
    enableBreaks = !enableBreaks;

    if (enableBreaks) {
      this.enableBreaks();
    } else {
      this.disableBreaks();
    }
  }

  public void setDriveVoltage(double leftvoltage, double rightvoltage) {
    left1.setVoltage(leftvoltage);
    right1.setVoltage(rightvoltage);
  }

  public void drive(double xspeed, double zrotation) {
    if(!isTurningEnabled){
      zrotation = 0;
    }
    //drivetrain.curvatureDrive(-xspeed, -zrotation, true);

    drivetrain.curvatureDrive(-slewFilter.calculate(xspeed), -zrotation, true);
  }

  public void autoDrive(double xspeed, double zrotation) {
    //todo invert zroate??
    drivetrain.arcadeDrive(xspeed, -zrotation);
    //drivetrain.arcadeDrive(slewFilter.calculate(xspeed), -zrotation);
  }

  public void autoDriveDifferential(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  public void stopDrive(){
    drivetrain.tankDrive(0, 0);
  }
  
  public double getAvergeVelocity() {
    return left1.getEncoder().getVelocity() + right1.getEncoder().getVelocity() / 2;
  }

  public double getMaxMPS() {
    return maxMPS;
  }

  public double getRightVelocity() {
    return right1.getEncoder().getVelocity();
  }

  public double getLeftVelocity() {
    return left1.getEncoder().getVelocity();
  }

  public boolean isBalanced() {
    // if within 2.5 deg
    return (Math.abs(gyro.getPitch()) < Constants.GyroConstants.balanceRange);
  }

  public double getRobotPositionOld() {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left1 (raw): ", left1.getEncoder().getPosition());
    SmartDashboard.putNumber("right1 (raw): ", right1.getEncoder().getPosition());
    SmartDashboard.putNumber("left1 speed (raw): ", left1.getEncoder().getVelocity());
    SmartDashboard.putNumber("right1 speed (raw): ", right1.getEncoder().getVelocity());
    //System.out.println("wheelspeed: " + left1.getEncoder().getVelocity() + " ctilt: " + gyro.getPitch() + "-pTilt: " + prevTilt + "\n =     "  + (gyro.getPitch() - prevTilt));
    SmartDashboard.putNumber("left1 (feet): ", Units.metersToFeet(left1.getEncoder().getPosition()));
    SmartDashboard.putNumber("right1 (feet): ", Units.metersToFeet(right1.getEncoder().getPosition()));

    SmartDashboard.putNumber("Robot Tilt", (int) gyro.getPitch());
    SmartDashboard.putBoolean("Is Balanced", isBalanced());
    SmartDashboard.putData("Gyro", gyro);

    SmartDashboard.putNumber("Robot FPS: ", Units.metersToFeet(getAvergeVelocity()));
    SmartDashboard.putNumber("Robot speed (raw): ", getAvergeVelocity());


    double mps = Math.abs(getAvergeVelocity());
    if (mps> maxMPS) {
      maxMPS = mps;
    }

    SmartDashboard.putNumber("maxFPS", Units.metersToFeet(getMaxMPS()));
    SmartDashboard.putNumber("maxMPS", getMaxMPS());


    double x = tx.getDouble(0.0);
    limeData.setTx(x);
    double y = ty.getDouble(0.0);
    limeData.setTy(y);
    double area = ta.getDouble(0.0);
    limeData.setTa(area);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

  }

  public LimeData getLimeData() {
    return limeData;
  }


  public void resetPosition() {

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
    System.out.println("Brakes");
    enableBreaks = true;
  }

  public void disableBreaks() {
    left1.setIdleMode(IdleMode.kCoast);
    left2.setIdleMode(IdleMode.kCoast);
    right1.setIdleMode(IdleMode.kCoast);
    right2.setIdleMode(IdleMode.kCoast);
    enableBreaks = false;
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

    public RobotPosition() {
      leftposition = 0;
      rightposition = 0;
      angle = 0;
      averagePosition = 0;
    }

    public RobotPosition(double l, double r, double a) {
      leftposition = l;
      rightposition = r;
      angle = a;
      averagePosition = (l + r) / 2;
    }
  }

  public class LimeData {
    private double tx = 0,ty = 0,ta = 0;

    public void setTx(double tx) {
      this.tx = tx;
    }

    public double getTx() {
      return tx;
    }
    public void setTy(double ty) {
      this.ty = ty;
    }

    public double getTy() {
      return ty;
    }
    public void setTa(double ta) {
      this.ta = ta;
    }

    public double getTa() {
      return ta;
    }

    public void setPipeline(int pipeline){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }
  }
}
