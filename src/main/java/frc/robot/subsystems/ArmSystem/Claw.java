// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class Claw extends SubsystemBase {
  
  private final DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticConstants.Claw.clawChannelForwards, PneumaticConstants.Claw.clawChannelBackwards);

  public Claw() {
    Compressor c = new Compressor(PneumaticsModuleType.REVPH);
    claw.set(Value.kForward);
    //c.disable();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Claw Piston: ", claw.get().equals(Value.kForward) ? "Opened" : "Closed");
  }

  
  public void toggle() {
    if (claw.get().equals(Value.kForward)){
      claw.set(Value.kReverse);
    } else{
      claw.set(Value.kForward);
    }
  }

  public void close() {
    claw.set(Value.kReverse);
  }

  public void open() {
    claw.set(Value.kForward);
  }
}
