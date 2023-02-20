// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoTEST extends SubsystemBase {
  /** Creates a new ServoTEST. */
  Servo servo = new Servo(9);
  public ServoTEST() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void servoToZero(){
    servo.setAngle(0);
  }

  public void servoTo90(){
    servo.setAngle(90);
  }

}
