// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mxp extends SubsystemBase {
  /** Creates a new Mxp. */
  AHRS ahrs;
  public float zAccel = 0;
  public Mxp() {
    ahrs = new AHRS(SPI.Port.kMXP); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      zAccel = ahrs.getRawAccelZ();
      System.out.println(zAccel);
  }
}
