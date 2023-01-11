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
  public TankDrive() {
    CANSparkMax left1 = new CANSparkMax(40, MotorType.kBrushless);
    CANSparkMax left2 = new CANSparkMax(41, MotorType.kBrushless);
    CANSparkMax right1 = new CANSparkMax(42, MotorType.kBrushless);
    CANSparkMax right2 = new CANSparkMax(43, MotorType.kBrushless);
    MotorControllerGroup leftgroup = new MotorControllerGroup(left1, left2);
    MotorControllerGroup rightgroup = new MotorControllerGroup(right1, right2);
    DifferentialDrive drivetrain = new DifferentialDrive(leftgroup, rightgroup);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
