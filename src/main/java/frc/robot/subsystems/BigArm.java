// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ArmSystemConstants;

public class BigArm extends SubsystemBase {
  /** Creates a new BigArm. */
  public final CANSparkMax bigArm = new CANSparkMax(ArmSystemConstants.bigArmCANID, MotorType.kBrushless);

  public final DigitalInput bigArmZero = new DigitalInput(Constants.ArmSystemConstants.bigArmZeroChannel);
  private final DoubleSolenoid brakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PneumaticConstants.BigArmBrake.channelForwards, Constants.PneumaticConstants.BigArmBrake.channelBackwards);

  public BigArm() {
    new Trigger(bigArm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(Commands.runOnce(this::zeroBigArm, this));
    bigArm.restoreFactoryDefaults();
    bigArm.getEncoder().setPositionConversionFactor(2);
    SmartDashboard.putNumber("BigArm speed: ", 0.3);
    bigArm.setIdleMode(IdleMode.kBrake);

  }

  public void zeroBigArm() {
    System.out.println("Zero Big Arm From: " + bigArm.getEncoder().getPosition());
    bigArm.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("BigArmZero: ", bigArmZero.get());
    SmartDashboard.putNumber("BigArm Angle: ", bigArm.getEncoder().getPosition());
    SmartDashboard.putString("Big ArmBrake State: ", brakePiston.get().toString());
  }
  public void bigArmForward(){
    if(bigArm.getEncoder().getPosition() >= ArmSystemConstants.BigArmAngle.maxAngle){
      bigArm.set(0);
    }
    else{
      double bigSpeed = SmartDashboard.getNumber("BigArm speed: ", 0);
      bigArmStart(bigSpeed);
    }
  }

  public void bigArmBackward(){
    if(bigArm.getEncoder().getPosition() <= ArmSystemConstants.BigArmAngle.minAngle){
      bigArm.set(0);
    }
    else{
      double bigSpeed = SmartDashboard.getNumber("BigArm speed: ", 0);
      bigArmStart(-bigSpeed);
    }
  }

  public void bigArmOff(){
    bigArmStop();
  }

  private void bigArmStop() {
    bigArm.set(0);
    this.enableBigArmBreak();
  }

  private void bigArmStart(double speed) {
    disableBigArmBreak();
    if (isBigArmBraked()) {
      bigArm.set(0);
    } else {
      bigArm.set(speed);
    }

  }

  public void enableBigArmBreak() {
    brakePiston.set(Value.kReverse);
  }

  public void disableBigArmBreak() {
    brakePiston.set(Value.kForward);
  }

  private boolean isBigArmBraked() { 
    return brakePiston.get() == (Value.kForward);
  }

  public double getBigArmPosition(){
    return  bigArm.getEncoder().getPosition();
   }
 
   public void setBigArm(double speed){
     bigArm.set(speed);
   }

   public CANSparkMax getBigArmController() {
    return bigArm;
  }
}
