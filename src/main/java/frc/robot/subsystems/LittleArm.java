// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmSystemConstants;

public class LittleArm extends SubsystemBase {
  /** Creates a new LittleArm. */
  public final CANSparkMax littleArm = new CANSparkMax(ArmSystemConstants.littleArmCANID, MotorType.kBrushless);

  public LittleArm() {
    new Trigger(littleArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(Commands.runOnce(this::zeroLittleArm, this));
    littleArm.restoreFactoryDefaults();
    littleArm.getEncoder().setPositionConversionFactor(2.3819);
    SmartDashboard.putNumber("LittleArm speed: ", 0.3);
    littleArm.setIdleMode(IdleMode.kBrake);


  }

  public void zeroLittleArm(){
    System.out.println("Zero Little Arm From: " + littleArm.getEncoder().getPosition());
    littleArm.getEncoder().setPosition(0);
  }

  public void littleArmForward(){
    double littleSpeed = SmartDashboard.getNumber("LittleArm speed: ", 0);
    littleArm.set(littleSpeed);
  }
  
  public void littleArmBackward(){
    double littleSpeed = SmartDashboard.getNumber("LittleArm speed: ", 0);
    littleArm.set(-littleSpeed);
  }

  public void littleArmOff(){
    littleArm.set(0);
  }

  public double getLittleArmPositon(){
    return littleArm.getEncoder().getPosition();
  }

  public void setLittleArm(double speed){
    littleArm.set(speed);
  }

  public CANSparkMax getLittleArmController() {
    return littleArm;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Little Angle: ", littleArm.getEncoder().getPosition());
    // This method will be called once per scheduler run

  }
}
