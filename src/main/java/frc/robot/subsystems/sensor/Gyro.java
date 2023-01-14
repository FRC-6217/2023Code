// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensor;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */

  private WPI_Pigeon2 pigeon= new WPI_Pigeon2(10);

  private final int ROLL_POS = 0, PITCH_POS = 1, YAW_POS = 2, NUM_POS = 3;
  private final int NUM_TAPS = 10;
  private double orientation[] = {0,0,0};
  private LinearFilter filters[] =  new LinearFilter[NUM_POS];
  public Gyro() {
    pigeon.reset();

    for(int i = 0; i < NUM_POS; i++) {
      System.out.println(i);
      filters[i] = LinearFilter.movingAverage(NUM_TAPS);
    }
  }
  

  //thomas todo
  public boolean isBalanced() {
    // if within 2.5 deg


    return true;
  }

  @Override
  public void periodic() {

    orientation[ROLL_POS] = filters[ROLL_POS].calculate(pigeon.getRoll());
    orientation[PITCH_POS] = filters[PITCH_POS].calculate(pigeon.getPitch());
    orientation[YAW_POS] = filters[YAW_POS].calculate(pigeon.getYaw());
    

    SmartDashboard.putNumber("Robot Tilt", orientation[PITCH_POS]);
    SmartDashboard.putBoolean("Is Balanced", isBalanced());
    SmartDashboard.putData("Gyro", pigeon);

  }

  public void reset(){

    pigeon.reset();

    for(int i = 0; i < NUM_POS; i++) {
      orientation[i] = 0;
      filters[i].reset();
    }

  }
  public double getPitch(){
    return orientation[PITCH_POS];
  }

  public double getYaw(){
    return orientation[YAW_POS];
  }
  public double getRoll(){
    return orientation[ROLL_POS];
  }
}
