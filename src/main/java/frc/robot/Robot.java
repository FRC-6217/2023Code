// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommands.EnableBigArmBrake;
import frc.robot.commands.AutoCommands.EnableBrakes;
import frc.robot.commands.DriveCommands.DisableBreaks;
import frc.robot.subsystems.TankDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // todo remove for competition Enable breaks on init, disable breaks on disable
    //new Trigger(this::isEnabled).negate().debounce(3).onTrue(new DisableBreaks(this.m_robotContainer.mTankDrive));
    //new Trigger(this::isEnabled).onTrue(Commands.runOnce(m_robotContainer.mTankDrive::enableBreaks, m_robotContainer.mTankDrive));
   // new Trigger(this::isEnabled).negate().debounce(3).whenActive(Commands.runOnce(this.m_robotContainer.mTankDrive::disabledbreaks, this.m_robotContainer.mTankDrive));
    SmartDashboard.putData(CommandScheduler.getInstance());

    EnableBigArmBrake armBrake = new EnableBigArmBrake(m_robotContainer.bigArm);
    armBrake.schedule();
    Commands.runOnce(m_robotContainer.mTankDrive::enableBreaks, m_robotContainer.mTankDrive).schedule();

    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
;
    //ToggldeClaw tClaw = new ToggldeClaw(m_robotContainer.armSystem, cState.OPEN);
   // tClaw.schedule();
   m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    //System.out.println(m_autonomousCommand.getName());
    // schedule the autonomous command (example)
   if (m_autonomousCommand != null) {
    m_autonomousCommand.schedule();
    }



    //m_robotContainer.getAutoCommandFactory().AlwaysDo().andThen(m_robotContainer.getAutoCommandFactory().getStep1()).andThen( m_robotContainer.getAutoCommandFactory().getStep2()).schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Command brakes = new EnableBrakes(m_robotContainer.mTankDrive);
    brakes.schedule();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
