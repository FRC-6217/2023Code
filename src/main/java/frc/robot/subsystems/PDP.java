// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PDPConstants;

public class PDP extends SubsystemBase {

  PowerDistribution pdp;

  /** Creates a new PDP. */
  public PDP() {

    pdp = new PowerDistribution(PDPConstants.ID, ModuleType.kRev);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(pdp);
  }
}
