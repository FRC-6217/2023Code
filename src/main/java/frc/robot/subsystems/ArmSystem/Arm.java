// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IArmConstants;

import com.revrobotics.SparkMaxLimitSwitch;


public class Arm extends SubsystemBase {

  private final CANSparkMax arm;
  private String name;
  AnalogInput pot;
  private IArmConstants constants;

  SlewRateLimiter slewRateLimiter;
  double maxAcc = 1;
  boolean useSlewRate = true;
  public Setpoints setpoints;

  public Arm(IArmConstants constants, boolean useSlewRate) {

    this.useSlewRate = useSlewRate;

    this.arm = new CANSparkMax(constants.getCANDid(), MotorType.kBrushless);
    arm.restoreFactoryDefaults();
    arm.setInverted(true);
    this.constants = constants;

    arm.setIdleMode(IdleMode.kBrake);
    this.name = constants.getName();

    new Trigger(arm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(Commands.runOnce(this::resetArmPosition, this));
    new Trigger(arm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)::isPressed).onTrue(Commands.runOnce(() -> System.out.println(name + " reverse limit hit"), this));
    pot = new AnalogInput(constants.getPotChannel());
    SmartDashboard.putNumber(name + " speed: ", 0.35);
   
    setPositionConversionFactor(constants.getPositionConversionFactor());

    slewRateLimiter = new SlewRateLimiter(maxAcc);
    SmartDashboard.putNumber(name + "slew rate", 1);

    setpoints = new Setpoints();
    setupKnownSetPoints();


  }

  public Arm(IArmConstants constants) {
    this(constants, false);
  }

  public void resetSlewRate() {
    maxAcc = SmartDashboard.getNumber(name + "slew rate", 1);
    slewRateLimiter = new SlewRateLimiter(maxAcc);

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
    return constants.getScaleFactor()*(pot.getVoltage()-constants.getOffset());
  }

  public void resetArmPosition(){
    System.out.println(name + " reset: angle: " + getAngle());
    arm.getEncoder().setPosition(0);
  }

  public void armConstantSpeedForwardFromDashBoard(){
    double speed;
    if (useSlewRate) {
      speed = slewRateLimiter.calculate(SmartDashboard.getNumber(name + " speed: ", 0));
    } else {
      speed = SmartDashboard.getNumber(name + " speed: ", 0);
    }
    
    arm.set(speed);
  }

  public void armConstantSpeedBackwardFromDashBoard(){
    double speed;
    if (useSlewRate) {
      speed = slewRateLimiter.calculate(-SmartDashboard.getNumber(name + " speed: ", 0));
    } else {
      speed = -SmartDashboard.getNumber(name + " speed: ", 0);
    }
    arm.set(speed);
  }

  public void armConstantSpeed(double speed) {
    if (useSlewRate) {
      speed = slewRateLimiter.calculate(speed);
    } else {

    }
    arm.set(speed);
  }

  public void stop(){
    arm.set(0);
    slewRateLimiter = new SlewRateLimiter(maxAcc);
  }

  public String getArmName() {
    return name;
  }



  @Override
  public void periodic() {
      SmartDashboard.putNumber(name + " angle: ", getAngle());
      SmartDashboard.putNumber(name + " Pot: ", pot.getVoltage());

      if (SmartDashboard.getNumber(name + " high Cube setpoint: ", constants.getHighCubeSetPoint()) != this.setpoints.highCube) {
        this.setpoints.highCube = SmartDashboard.getNumber(name + " high Cube setpoint: ", constants.getHighCubeSetPoint());
      }
      if (SmartDashboard.getNumber(name + " mid Cube setpoint: ", constants.getMidCubeSetPoint()) != this.setpoints.midCube) {
        this.setpoints.midCube = SmartDashboard.getNumber(name +  "mid Cube setpoint: ", constants.getHighCubeSetPoint());
      }
      if (SmartDashboard.getNumber(name + " low Cube setpoint: ", constants.getPickUpCubeSetPoint()) != this.setpoints.lowCube) {
        this.setpoints.lowCube = SmartDashboard.getNumber(name + " low Cube setpoint: ", constants.getPickUpCubeSetPoint());
      }

      if (SmartDashboard.getNumber(name + " high Cube setpoint: ", constants.getHighConeSetPoint()) != this.setpoints.highCube) {
        this.setpoints.highCube = SmartDashboard.getNumber(name + " high Cube setpoint: ", constants.getHighConeSetPoint());
      }
      if (SmartDashboard.getNumber(name + " mid Cube setpoint: ", constants.getMidConeSetPoint()) != this.setpoints.highCube) {
        this.setpoints.midCube = SmartDashboard.getNumber(name + " mid Cube setpoint: ", constants.getMidConeSetPoint());
      }
      if (SmartDashboard.getNumber(name + " low Cube setpoint: ", constants.getPickUpConeSetPoint()) != this.setpoints.highCube) {
        this.setpoints.lowCube = SmartDashboard.getNumber(name + " low Cube setpoint: ", constants.getPickUpConeSetPoint());
      }

      if (SmartDashboard.getNumber(name + " substation setpoint: ", constants.getSubstationSetPoint()) != this.setpoints.highCube) {
        this.setpoints.substation = SmartDashboard.getNumber(name + " substation setpoint: ", constants.getSubstationSetPoint());
      }

      if (SmartDashboard.getNumber(name + " saftey setpoint: ", constants.getSafteySetPoint()) != this.setpoints.saftey) {
        this.setpoints.saftey = SmartDashboard.getNumber(name + " saftey setpoint: ", constants.getSafteySetPoint());
      }
  }


  private void setupKnownSetPoints() {

   this.setpoints.highCube = constants.getHighCubeSetPoint();
   this.setpoints.midCube = constants.getMidCubeSetPoint();
   this.setpoints.lowCube = constants.getPickUpCubeSetPoint();

   this.setpoints.highCone = constants.getHighConeSetPoint();
   this.setpoints.midCone = constants.getMidConeSetPoint();
   this.setpoints.lowCone = constants.getPickUpConeSetPoint();

   this.setpoints.substation = constants.getSubstationSetPoint();

   this.setpoints.saftey = constants.getSafteySetPoint();

    SmartDashboard.putNumber(name + " high Cube setpoint: ", constants.getHighCubeSetPoint());
    SmartDashboard.putNumber(name + " mid Cube setpoint: ", constants.getMidCubeSetPoint());
    SmartDashboard.putNumber(name + " low Cube setpoint: ", constants.getPickUpCubeSetPoint());

    SmartDashboard.putNumber(name + " high Cone setpoint: ", constants.getHighConeSetPoint());
    SmartDashboard.putNumber(name + " mid Cone setpoint: ", constants.getMidConeSetPoint());
    SmartDashboard.putNumber(name + " low Cone setpoint: ", constants.getPickUpConeSetPoint());


    SmartDashboard.putNumber(name + " substation setpoint: ", constants.getSubstationSetPoint());

    SmartDashboard.putNumber(name + " saftey setpoint: ", constants.getSafteySetPoint());
  }



  public class Setpoints {

    public double highCube;
    public double midCube;
    public double lowCube;

    public double highCone;
    public double midCone;
    public double lowCone;

    public double substation;


    public double saftey;
  }

}
