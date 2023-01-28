// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.NumberConstants;

public class TankDrive extends SubsystemBase {
  /** Creates a new TankDrive. */
  CANSparkMax left1;
  CANSparkMax left2;
  CANSparkMax right1;
  CANSparkMax right2;
  MotorControllerGroup leftgroup;
  MotorControllerGroup rightgroup;
  DifferentialDrive drivetrain;

  //todo
  private final double VELOCITY_CONVERSION_FACTOR  = NumberConstants.WHEEL_DIAMETER*NumberConstants.PI/12/60;
private final double POSITION_CONVERSION_FACTOR = NumberConstants.WHEEL_DIAMETER * NumberConstants.PI;

  public TankDrive() {

     left1 = new CANSparkMax(DriveTrainConstants.LEFT_1, MotorType.kBrushless);
     left2 = new CANSparkMax(DriveTrainConstants.LEFT_2, MotorType.kBrushless);

     right1 = new CANSparkMax(DriveTrainConstants.RIGHT_1, MotorType.kBrushless);
     right2 = new CANSparkMax(DriveTrainConstants.RIGHT_2, MotorType.kBrushless);

     leftgroup = new MotorControllerGroup(left1, left2);
     rightgroup = new MotorControllerGroup(right1, right2);

     leftgroup.setInverted(true);
     rightgroup.setInverted(false);
     drivetrain = new DifferentialDrive(leftgroup, rightgroup);
     

     left1.setIdleMode(IdleMode.kBrake);
     left2.setIdleMode(IdleMode.kBrake);
     right1.setIdleMode(IdleMode.kBrake);
     right2.setIdleMode(IdleMode.kBrake);

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


     
  }

  public void drive(double xspeed, double zrotation){
   drivetrain.curvatureDrive(xspeed, zrotation, true);
    //drivetrain.arcadeDrive(xspeed, zrotation);
  }

  public int getAvergeFPS() {
    return (int) Math.round(left1.getEncoder().getVelocity() + right1.getEncoder().getVelocity() / 2);
  }

  public double getRobotPosition() {
    return (left1.getEncoder().getPosition());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("robotposition",getRobotPosition());

    SmartDashboard.putNumber("left1", left1.getEncoder().getPosition());
    SmartDashboard.putNumber("left2", left2.getEncoder().getPosition());
    SmartDashboard.putNumber("right1", right1.getEncoder().getPosition());
    SmartDashboard.putNumber("right2", right2.getEncoder().getPosition());

    SmartDashboard.putNumber("Robot FPS", getAvergeFPS());
  }

  public void resetPosition() {
  left1.getEncoder().setPosition(0);
  left2.getEncoder().setPosition(0);
  right1.getEncoder().setPosition(0);
  right2.getEncoder().setPosition(0);

  }
}
