// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.sensor.Gyro;
import frc.robot.subsystems.sensor.Xcelerameter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final TankDrive mTankDrive = new TankDrive();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick driveJoystick = new CommandJoystick(0);
  private final Gyro gyro = new Gyro();
  private final PDP pdp = new PDP();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
        CommandScheduler.getInstance().setDefaultCommand(mTankDrive, new TeleopDrive(mTankDrive, driveJoystick));

    driveJoystick.button(12).whileTrue(new PIDCommand(
      new PIDController(0,0,0),
      // Close the loop on the turn rate
      gyro::getRoll,
      // Setpoint is 0
      (double)0,
      // Pipe the output to the turning controls
      output -> mTankDrive.drive(output, 0),
      // Require the robot drive
        mTankDrive));
        
   /* new JoystickButton(driveJoystick, 12)
    .whileTrue(
        new PIDCommand(
            new PIDController(
                DriveConstants.kStabilizationP,
                DriveConstants.kStabilizationI,
                DriveConstants.kStabilizationD),
            // Close the loop on the turn rate
            m_robotDrive::getTurnRate,
            // Setpoint is 0
            0,
            // Pipe the output to the turning controls
            output -> m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), output),
            // Require the robot drive
                m_robotDrive));
 */

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
