// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensor;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Xcelerameter extends SubsystemBase {
  /** Creates a new Aceelerameter. */
  Accelerometer accelerometer;

  int Counter = 0;
  LinearFilter filter;
  double filteraccel = 0;

  public Xcelerameter() {
    filter = LinearFilter.movingAverage(10);
    accelerometer = new BuiltInAccelerometer();
  }

  @Override
  public void periodic() {
    double z = accelerometer.getZ() > 1 ? 1 : accelerometer.getZ();
    filteraccel = filter.calculate(z);
    double degrees = Math.toDegrees(Math.acos(filteraccel));
    // This method will be called once per scheduler run

  }

  public double getRobotTilt() {
    // spiderman
    return filteraccel;
  }
}
