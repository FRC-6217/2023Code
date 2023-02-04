// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SimpleMotorController extends SubsystemBase {
  
  private final String speedKey = " speed:";
  private final String stateKey = " state:";
  /** Creates a new SimpleMotorController. */
  private CANSparkMax controller;
  String name;

  public SimpleMotorController(int sparkMaxID, String name) {
    controller = new CANSparkMax(sparkMaxID, MotorType.kBrushless);
    this.name = name;

    SmartDashboard.putNumber(name + speedKey, 0);
    SmartDashboard.putString(name + stateKey, "init");

  }

  public void on() {
    controller.set(SmartDashboard.getNumber(name + speedKey, 0));
    SmartDashboard.putString(name + stateKey, "on");
  }

  public void off() {
    controller.set(0);
    SmartDashboard.putString(name + stateKey, "off");
  }

  @Override
  public void periodic() {


    
  }
}
