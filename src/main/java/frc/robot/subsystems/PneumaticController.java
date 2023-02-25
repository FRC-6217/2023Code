// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.PneumaticConstants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticController extends SubsystemBase {
  /** Creates a new PneumaticController. */
 // Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
 PneumaticHub pHub = new PneumaticHub();
  DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticConstants.Claw.clawChannelForwards, PneumaticConstants.Claw.clawChannelBackwards);

  public PneumaticController() {
   // compressor.enableDigital();
  }

  @Override
  public void periodic() {
  //  SmartDashboard.putString(PneumaticConstants.compressorKey + "state: ", compressor.isEnabled() ? "on" : "off");
  //  SmartDashboard.putString(PneumaticConstants.compressorKey + "pressure switch: ", compressor.getPressureSwitchValue() ? "on" : "off");
    SmartDashboard.putString(PneumaticConstants.Claw.key + "state: ", claw.get().equals(Value.kForward) ? "open" : "closed");

  }

  public void initalize() {
    claw.set(Value.kForward);
  }

  public void toggleClaw() {
    if (claw.get().equals(Value.kForward)){
    claw.set(Value.kReverse);
  } else{
    claw.set(Value.kForward);
  }
  }

}
