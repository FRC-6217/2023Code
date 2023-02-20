// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DMA;
import edu.wpi.first.wpilibj.DMASample;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PotentiameterTest extends SubsystemBase {
  /** Creates a new PotentiameterTest. */
  private DMA dma = new DMA();
  DMASample dmaSample = new DMASample();
  AnalogInput ai = new AnalogInput(0);
  private DigitalOutput m_dmaTrigger = new DigitalOutput(2);;
  public PotentiameterTest() {
    dma.addAnalogInput(ai);
    dma.setExternalTrigger(m_dmaTrigger, false, true);
    m_dmaTrigger.set(true);
    dma.start(1024);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_dmaTrigger.set(false);
    DMASample.DMAReadStatus readStatus = dmaSample.update(dma, Units.millisecondsToSeconds(1));
    m_dmaTrigger.set(true);
   // if (readStatus == DMASample.DMAReadStatus.kOk) {
      double analogVoltage = dmaSample.getAnalogInputVoltage(ai);
      SmartDashboard.putNumber("Input", analogVoltage);
      SmartDashboard.putData(ai);
 //   }else {
   //   System.out.println("oh no");
   // }
  }
}
