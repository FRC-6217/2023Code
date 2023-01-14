// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  private final double VELOCITY_CONVERSION_FACTOR  = 1;


  public TankDrive() {

     left1 = new CANSparkMax(40, MotorType.kBrushless);
     left2 = new CANSparkMax(41, MotorType.kBrushless);
     right1 = new CANSparkMax(42, MotorType.kBrushless);
     right2 = new CANSparkMax(43, MotorType.kBrushless);
     leftgroup = new MotorControllerGroup(left1, left2);
     rightgroup = new MotorControllerGroup(right1, right2);
     drivetrain = new DifferentialDrive(leftgroup, rightgroup);
     leftgroup.setInverted(true);

     left1.setIdleMode(IdleMode.kBrake);
     left2.setIdleMode(IdleMode.kBrake);
     right1.setIdleMode(IdleMode.kBrake);
     right2.setIdleMode(IdleMode.kBrake);

     left1.getEncoder().setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
     left2.getEncoder().setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
     right1.getEncoder().setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
     right2.getEncoder().setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
  }

  public void drive(double xspeed, double zrotation){
    drivetrain.arcadeDrive(xspeed, zrotation);
  }

  public double getAvergeFPS() {
    return Math.round(left1.getEncoder().getVelocity() + right1.getEncoder().getVelocity() / 2);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot FPS", getAvergeFPS());
  }
}
