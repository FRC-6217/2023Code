// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IArmConstants;

public class BigArm extends Arm {

  private final DoubleSolenoid brakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PneumaticConstants.BigArmBrake.channelForwards, Constants.PneumaticConstants.BigArmBrake.channelBackwards);

  public BigArm(IArmConstants constants) {
    super(constants);
    this.getController().setInverted(true);
  }

  public void enableBigArmBreak() {
    brakePiston.set(Value.kReverse);
  }

  public void disableBigArmBreak() {
    brakePiston.set(Value.kForward);
  }

  @Override
  public void armConstantSpeedForwardFromDashBoard(){
    disableBigArmBreak();
    super.armConstantSpeedForwardFromDashBoard();

  }

  @Override
  public void armConstantSpeedBackwardFromDashBoard(){
    disableBigArmBreak();
    super.armConstantSpeedBackwardFromDashBoard();
  }

  @Override
  public void armConstantSpeed(double speed) {
    disableBigArmBreak();
    super.armConstantSpeed(speed);
  }

  @Override
  public void periodic(){
    super.periodic();
    SmartDashboard.putString("Big Arm Piston: ", brakePiston.get().equals(Value.kForward) ? "Disable" : "Enable");
  }

  @Override
  public void stop(){
    super.stop();
    enableBigArmBreak();
  }

}
