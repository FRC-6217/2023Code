// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.PneumaticConstants;

public class ArmSystem extends SubsystemBase {
  /** Creates a new ArmSystem. */
  public final CANSparkMax bigArm = new CANSparkMax(Constants.ArmSystemConstants.bigArmCANID, MotorType.kBrushless);
  public final CANSparkMax littleArm = new CANSparkMax(Constants.ArmSystemConstants.littleArmCANID, MotorType.kBrushless);
  
  PneumaticHub pHub = new PneumaticHub();
  private final DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticConstants.Claw.clawChannelForwards, PneumaticConstants.Claw.clawChannelBackwards);
  private final DoubleSolenoid brakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PneumaticConstants.BigArmBrake.channelForwards, Constants.PneumaticConstants.BigArmBrake.channelBackwards);

  public final DigitalInput bigArmZero = new DigitalInput(Constants.ArmSystemConstants.bigArmZeroChannel);


  private final double bigArmMaxAngle = 40;
  private final double bigArmMinAngle = -50;

  // todo
  private final double littleArmMinAngle = 0;
  private final double littleArmMaxAngle = 0;


  public ArmSystem() {
    new Trigger(bigArmZero::get).onTrue(Commands.runOnce(this::zeroBigArm, this));

    bigArm.getEncoder().setPositionConversionFactor(2);
    littleArm.getEncoder().setPositionConversionFactor(40);

    SmartDashboard.putNumber("LittleArm speed: ", 0.3);
    SmartDashboard.putNumber("BigArm speed: ", 0.3);

   // SmartDashboard.putData(brakePiston);

  }


  public void zeroBigArm() {
    System.out.println("Zero Big Arm From: " + bigArm.getEncoder().getPosition());
    bigArm.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("BigArmZero: ", bigArmZero.get());
    SmartDashboard.putNumber("BigArm Angle: ", bigArm.getEncoder().getPosition());
    SmartDashboard.putNumber("Little Angle: ", littleArm.getEncoder().getPosition());
    SmartDashboard.putString("Big ArmBrake State: ", brakePiston.get().toString());
  }

  public void littleArmForward(){
    double littleSpeed = SmartDashboard.getNumber("LittleArm speed: ", 0);
    littleArm.set(littleSpeed);
  }
  public void littleArmBackward(){
    double littleSpeed = SmartDashboard.getNumber("LittleArm speed: ", 0);
    littleArm.set(-littleSpeed);
  }
  public void bigArmForward(){
    if(bigArm.getEncoder().getPosition() >= bigArmMaxAngle){
      bigArm.set(0);
    }
    else{
      double bigSpeed = SmartDashboard.getNumber("BigArm speed: ", 0);
      bigArmStart(bigSpeed);
    }
  }

  public void bigArmBackward(){
    if(bigArm.getEncoder().getPosition() <= bigArmMinAngle){
      bigArm.set(0);
    }
    else{
      double bigSpeed = SmartDashboard.getNumber("BigArm speed: ", 0);
      bigArmStart(-bigSpeed);
    }

    
  }

  public void littleArmOff(){
    littleArm.set(0);
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
      System.out.println("brakes engaged?");
      bigArm.set(0);
    } else {
      System.out.println("brakes disabled");
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

  public void toggleClaw() {
    /*
    if (claw.get().equals(Value.kForward)){
      claw.set(Value.kReverse);
    } else{
      claw.set(Value.kForward);
    }
    */
    claw.toggle();
  }
}
