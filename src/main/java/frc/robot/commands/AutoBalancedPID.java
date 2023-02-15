// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TankDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalancedPID extends PIDCommand {
  /** Creates a new AutoBalancedPID. */ 
  TankDrive tankDrive;
  public AutoBalancedPID(TankDrive tankDrive) {
    super(
        // The controller that the command will use
        new PIDController(0.032000, 0, -0.005000),
        // This should return the measurement
        () -> tankDrive.getGyro().getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
        tankDrive.autoDrive(-output, 0);
        });
        addRequirements(tankDrive);
        this.tankDrive = tankDrive;
        SmartDashboard.putNumber(" pvalue", 0);
        SmartDashboard.putNumber(" ivalue", 0);
        SmartDashboard.putNumber(" dvalue", 0);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }
  @Override 
  public void initialize() {
    super.initialize();
    m_controller.setP(SmartDashboard.getNumber(" pvalue", 0)); 
    m_controller.setI(SmartDashboard.getNumber(" ivalue", 0)); 
    m_controller.setD(SmartDashboard.getNumber(" dvalue", 0)); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  @Override
  public void end(boolean I) {
    System.out.println("hi");
    super.end(I);
    tankDrive.autoDrive(0, 0);
  }
  @Override
  public void execute() {
    super.execute();
    //System.out.println(" output: " + this.m_useOutput);
  }
}
