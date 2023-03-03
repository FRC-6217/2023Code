// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ArmSystemConstants;
import frc.robot.Constants.PneumaticConstants;

public class ArmSystem extends SubsystemBase {
  /** Creates a new ArmSystem. */

  public static enum ARM_SELECTION {
    BIG_ARM,
    LITTLE_ARM,
  };


  Position currentPostion = Position.WITH_OBJECT_HOME;

  public static enum Position {
    WITH_OBJECT_HOME,
    WITH_OBJECT_SAFETY,
    GROUND_PICK_UP,
    SUBSTATION_PICK_UP,
    HIGH_SAFTEY_POSITION_FRONT,
    HIGH_SAFTEY_POSITION_BACK,
    HIGH_DROP_OFF_CONE,
    HIGH_DROP_OFF_CUBE,
    MID_DROP_OFF_CONE,
    MID_DROP_OFF_CUBE,
    LOW_DROP_OFF_BOTH,
    SAFE_POSITION,
    HOME_POSITION,
  };

  public final CANSparkMax bigArm = new CANSparkMax(ArmSystemConstants.bigArmCANID, MotorType.kBrushless);
  public final CANSparkMax littleArm = new CANSparkMax(ArmSystemConstants.littleArmCANID, MotorType.kBrushless);
  
  PneumaticHub pHub = new PneumaticHub();
  private final DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticConstants.Claw.clawChannelForwards, PneumaticConstants.Claw.clawChannelBackwards);
  private final DoubleSolenoid brakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PneumaticConstants.BigArmBrake.channelForwards, Constants.PneumaticConstants.BigArmBrake.channelBackwards);

  public final DigitalInput bigArmZero = new DigitalInput(Constants.ArmSystemConstants.bigArmZeroChannel);


  Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);


  //private DMA dma = new DMA();
  //DMASample dmaSample = new DMASample();
  ///AnalogInput analogLittleArmInput = new AnalogInput(ArmSystemConstants.littleArmPotAnalogInChannel);
  //private DigitalOutput m_dmaTrigger = new DigitalOutput(ArmSystemConstants.DMA_DIO_INPUT_CHANNEL);


 // private double littleArmAngle = 0;



  public ArmSystem() {
    // Trigger to zero out big arm
    //new Trigger(bigArmZero::get).onTrue(Commands.runOnce(this::zeroBigArm, this));
    //new Trigger(littleArm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(Commands.runOnce(this::zeroLittleArm, this));
    new Trigger(littleArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(Commands.runOnce(this::zeroLittleArm, this));
    new Trigger(bigArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(Commands.runOnce(this::zeroBigArm, this));
    new Trigger(bigArm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(new PrintCommand(" reverse Limit hit"));
    bigArm.restoreFactoryDefaults();
    littleArm.restoreFactoryDefaults();

    bigArm.getEncoder().setPositionConversionFactor(2);
    littleArm.getEncoder().setPositionConversionFactor(2.3819);
    //compressor.disable();
    SmartDashboard.putNumber("LittleArm speed: ", 0.3);
    SmartDashboard.putNumber("BigArm speed: ", 0.3);
/*
    dma.addAnalogInput(analogLittleArmInput);
    dma.setExternalTrigger(m_dmaTrigger, false, true);
    m_dmaTrigger.set(true);
    dma.start(1024);
*/
    bigArm.setIdleMode(IdleMode.kBrake);
    littleArm.setIdleMode(IdleMode.kBrake);
  }

  public void bigArmLimit() {
    System.out.println("big arm hit limmit");
  }

  public void zeroBigArm() {
    System.out.println("Zero Big Arm From: " + bigArm.getEncoder().getPosition());
    bigArm.getEncoder().setPosition(0);
  }

  public void zeroLittleArm(){
    System.out.println("Zero Little Arm From: " + littleArm.getEncoder().getPosition());
    littleArm.getEncoder().setPosition(0);
  }


  @Override
  public void periodic() {

    //m_dmaTrigger.set(false);
    //DMASample.DMAReadStatus readStatus = dmaSample.update(dma, Units.millisecondsToSeconds(1));
    //m_dmaTrigger.set(true);



    /*if (readStatus == DMASample.DMAReadStatus.kOk) {
      double analogVoltage = dmaSample.getAnalogInputVoltage(analogLittleArmInput);
      littleArmAngle =  this.analogInputToAngle(analogVoltage);
    }*/

    SmartDashboard.putBoolean("BigArmZero: ", bigArmZero.get());
    SmartDashboard.putNumber("BigArm Angle: ", bigArm.getEncoder().getPosition());
    // calibrate position
    SmartDashboard.putNumber("Little Angle: ", littleArm.getEncoder().getPosition());
    SmartDashboard.putString("Big ArmBrake State: ", brakePiston.get().toString());

  }

  public void littleArmForward(){
    double littleSpeed = SmartDashboard.getNumber("LittleArm speed: ", 0);
    littleArm.set(littleSpeed);
   // littleArm.set(littleSpeed);

  }
  public void littleArmBackward(){
    double littleSpeed = SmartDashboard.getNumber("LittleArm speed: ", 0);
    littleArm.set((-littleSpeed));
    //littleArm.set(-littleSpeed);
  }
  public void bigArmForward(){
    System.out.println("Big Arm Forward");
    /*
    if(bigArm.getEncoder().getPosition() >= ArmSystemConstants.BigArmAngle.maxAngle){
      bigArm.set(0);
    }
    else
    */{
      double bigSpeed = SmartDashboard.getNumber("BigArm speed: ", 0);
      bigArmStart(bigSpeed);
    }
  }

  public void bigArmBackward(){

    /* 
    if(bigArm.getEncoder().getPosition() <= ArmSystemConstants.BigArmAngle.minAngle){
      bigArm.set(0);
    }
    else
    */
    {
      double bigSpeed = SmartDashboard.getNumber("BigArm speed: ", 0);
      bigArmStart(-bigSpeed);
    }
  }

  public void littleArmOff(){
    littleArm.set(0);
  }

  public void bigArmOff(){
    System.out.println("Big Arm Stop");
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
      //bigArm.set(speed);
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
  
  }

  private double analogInputToAngle(double input) {
    // todo
    return input;
  }

  public double getLittleArmPositon(){
    return littleArm.getEncoder().getPosition();
  }

  public double getBigArmPosition(){
   return  bigArm.getEncoder().getPosition();
  }

  public void setBigArm(double speed){
    bigArm.set(speed);
  }

  public void setLittleArm(double speed){
    littleArm.set(speed);
  }

  public  ArmPosition getSetPointsFor(Position position) {
    switch(position) {
      case WITH_OBJECT_HOME:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.Home, Constants.ArmSystemConstants.BigArmAngle.Home);
      case WITH_OBJECT_SAFETY:
          return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.ObjectSafety,Constants.ArmSystemConstants.BigArmAngle.ObjectSafety);
      case GROUND_PICK_UP:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.GroundPickUp,Constants.ArmSystemConstants.BigArmAngle.GroundPickUp);
      case SUBSTATION_PICK_UP:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.SubstationPickUp,Constants.ArmSystemConstants.BigArmAngle.SubstationPickUp);
      case HIGH_SAFTEY_POSITION_FRONT:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.HighSafteyPositionFront,Constants.ArmSystemConstants.BigArmAngle.HighSafteyPositionFront);
      case HIGH_SAFTEY_POSITION_BACK:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.HighSafteyPositionBack,Constants.ArmSystemConstants.BigArmAngle.HighSafteyPositionBack);
      case HIGH_DROP_OFF_CONE:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.High_Drop_Off_Cone,Constants.ArmSystemConstants.BigArmAngle.High_Drop_Off_Cone);
      case HIGH_DROP_OFF_CUBE:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.High_Drop_Off_Cube,Constants.ArmSystemConstants.BigArmAngle.High_Drop_Off_Cube);
      case MID_DROP_OFF_CONE:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.Mid_Drop_Off_Cone,Constants.ArmSystemConstants.BigArmAngle.Mid_Drop_Off_Cone);
      case MID_DROP_OFF_CUBE:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.Mid_Drop_Off_Cube,Constants.ArmSystemConstants.BigArmAngle.Mid_Drop_Off_Cube);
      case LOW_DROP_OFF_BOTH:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.Low_Drop_Off_Both,Constants.ArmSystemConstants.BigArmAngle.Low_Drop_Off_Both);
      case SAFE_POSITION:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.Safe_Position,Constants.ArmSystemConstants.BigArmAngle.Safe_Position);
      case HOME_POSITION:
        return new ArmPosition(Constants.ArmSystemConstants.LittleArmAngle.Home_Position,Constants.ArmSystemConstants.BigArmAngle.Home_Position);
      default:
      System.out.println("Unexpected Arm State");
      return null;
    }
  }

  public CANSparkMax getBigArmController() {
    return bigArm;
  }

  public CANSparkMax getLittleArmController() {
    return littleArm;
  }

  public Position getCurrentPosition(){
    return currentPostion;
  }

  public void setPostion(Position position) {
    currentPostion = position;
  }

  public class ArmPosition{
    
    public double littleArmSetPoint;
    public double bigArmSetPoint;

    public ArmPosition(double littleArmPosition, double bigArmPosition){
      this.littleArmSetPoint = littleArmPosition;
      this.bigArmSetPoint = bigArmPosition;
    }

    public ArmPosition(){
      littleArmSetPoint = 0;
      bigArmSetPoint = 0;
    }
  }
}
