// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DMA;
import edu.wpi.first.wpilibj.DMASample;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ArmSystemConstants;
import frc.robot.Constants.PneumaticConstants;

public class ArmSystem extends SubsystemBase {
  /** Creates a new ArmSystem. */
  public final CANSparkMax bigArm = new CANSparkMax(ArmSystemConstants.bigArmCANID, MotorType.kBrushless);
  public final CANSparkMax littleArm = new CANSparkMax(ArmSystemConstants.littleArmCANID, MotorType.kBrushless);
  
  PneumaticHub pHub = new PneumaticHub();
  private final DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticConstants.Claw.clawChannelForwards, PneumaticConstants.Claw.clawChannelBackwards);
  private final DoubleSolenoid brakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PneumaticConstants.BigArmBrake.channelForwards, Constants.PneumaticConstants.BigArmBrake.channelBackwards);

  public final DigitalInput bigArmZero = new DigitalInput(Constants.ArmSystemConstants.bigArmZeroChannel);
  Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  private DMA dma = new DMA();
  DMASample dmaSample = new DMASample();
  AnalogInput analogLittleArmInput = new AnalogInput(ArmSystemConstants.littleArmPotAnalogInChannel);
  private DigitalOutput m_dmaTrigger = new DigitalOutput(ArmSystemConstants.DMA_DIO_INPUT_CHANNEL);
  
  private final double bigArmMaxAngle = 40;
  private final double bigArmMinAngle = -50;

  // todo
  private final double littleArmMinAngle = 0;
  private final double littleArmMaxAngle = 0;


  private double littleArmAngle = 0;


  public ArmSystem() {
    // Trigger to zero out big arm
    new Trigger(bigArmZero::get).onTrue(Commands.runOnce(this::zeroBigArm, this));

    bigArm.getEncoder().setPositionConversionFactor(2);
    littleArm.getEncoder().setPositionConversionFactor(40);
    //compressor.disable();
    SmartDashboard.putNumber("LittleArm speed: ", 0.3);
    SmartDashboard.putNumber("BigArm speed: ", 0.3);

    dma.addAnalogInput(analogLittleArmInput);
    dma.setExternalTrigger(m_dmaTrigger, false, true);
    m_dmaTrigger.set(true);
    dma.start(1024);

  }


  public void zeroBigArm() {
    System.out.println("Zero Big Arm From: " + bigArm.getEncoder().getPosition());
    bigArm.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {

    m_dmaTrigger.set(false);
    DMASample.DMAReadStatus readStatus = dmaSample.update(dma, Units.millisecondsToSeconds(1));
    m_dmaTrigger.set(true);

    if (readStatus == DMASample.DMAReadStatus.kOk) {
      double analogVoltage = dmaSample.getAnalogInputVoltage(analogLittleArmInput);
      littleArmAngle =  this.analogInputToAngle(analogVoltage);
    }

    SmartDashboard.putBoolean("BigArmZero: ", bigArmZero.get());
    SmartDashboard.putNumber("BigArm Angle: ", bigArm.getEncoder().getPosition());
    SmartDashboard.putNumber("Little Angle: ", littleArmAngle);
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
    
    if (claw.get().equals(Value.kForward)){
      claw.set(Value.kReverse);
    } else{
      claw.set(Value.kForward);
    }
    
    //claw.toggle();
  }


  private double analogInputToAngle(double input) {
    // todo
    return input;
  }
}
