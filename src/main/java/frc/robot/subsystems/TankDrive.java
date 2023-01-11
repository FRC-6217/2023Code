// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
  public TankDrive() {
     left1 = new CANSparkMax(40, MotorType.kBrushless);
     left2 = new CANSparkMax(41, MotorType.kBrushless);
     right1 = new CANSparkMax(42, MotorType.kBrushless);
     right2 = new CANSparkMax(43, MotorType.kBrushless);
     leftgroup = new MotorControllerGroup(left1, left2);
     rightgroup = new MotorControllerGroup(right1, right2);
     drivetrain = new DifferentialDrive(leftgroup, rightgroup);
     leftgroup.setInverted(true);
  }

  public void drive(double xspeed, double zrotation){
    drivetrain.arcadeDrive(xspeed, zrotation);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
