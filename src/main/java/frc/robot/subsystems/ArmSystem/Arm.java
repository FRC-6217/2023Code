// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IArmConstants;

import com.revrobotics.SparkMaxLimitSwitch;


public class Arm extends SubsystemBase {

  private final CANSparkMax arm;
  private String name;

  private IArmConstants constants;

  public Arm(IArmConstants constants) {

    this.arm = new CANSparkMax(constants.getCANDid(), MotorType.kBrushless);
    arm.setInverted(true);
    this.constants = constants;
    arm.restoreFactoryDefaults();
    arm.setIdleMode(IdleMode.kBrake);
    this.name = constants.getName();

    new Trigger(arm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(Commands.runOnce(this::resetArmPosition, this));
    new Trigger(arm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(Commands.runOnce(() -> System.out.println(name + " reverse limit hit"), this));
    

    SmartDashboard.putNumber(arm + " speed: ", 0.3);
    setPositionConversionFactor(constants.getPositionConversionFactor());

  }

  public IArmConstants getConstants() {
    return constants;
  }

  public CANSparkMax getController() {
    return arm;
  }

  public String getName() {
    return name;
  }

  public void setPositionConversionFactor(double factor) {
    arm.getEncoder().setPositionConversionFactor(factor);
  }


  public double getAngle() {
    return arm.getEncoder().getPosition();
  }

  public void resetArmPosition(){
    System.out.println(name + " old angle: " + getAngle() + " new angle: 0");
    arm.getEncoder().setPosition(0);
  }

  public void armConstantSpeedForwardFromDashBoard(){
    double speed = SmartDashboard.getNumber(arm + " speed: ", 0);
    arm.set(speed);
  }

  public void armConstantSpeedBackwardFromDashBoard(){
    double speed = SmartDashboard.getNumber(arm + " speed: ", 0);
    arm.set(-speed);
  }

  public void armConstantSpeed(double speed) {
    arm.set(speed);
  }

  public void stop(){
    arm.set(0);
  }

  public String getArmName() {
    return name;
  }



  @Override
  public void periodic() {
        SmartDashboard.putNumber(name + " angle: ", getAngle());
  }


}
