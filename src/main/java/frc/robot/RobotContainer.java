// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PersistenceData;
import frc.robot.commands.ArmCommands.ArmPositions;
import frc.robot.commands.AutoCommands.AutoBalanceBangBang;
import frc.robot.commands.AutoCommands.AutoBalancedPID;
import frc.robot.commands.AutoCommands.AutoCommandFactory;
import frc.robot.commands.AutoCommands.DriveToDistanceInches;
import frc.robot.commands.AutoCommands.DriveUntilUnBalanced;
import frc.robot.commands.AutoCommands.DriveUntilUnBalanced.Direction;
import frc.robot.commands.DriveCommands.CancelArms;
import frc.robot.commands.DriveCommands.CancelDriveTrain;
import frc.robot.commands.DriveCommands.DriveToGamePiece;
import frc.robot.commands.DriveCommands.DriveToSubStationPickUp;
import frc.robot.commands.DriveCommands.StayPutAllDOF;
import frc.robot.commands.DriveCommands.TeleopDrive;
import frc.robot.commands.DriveCommands.DriveToGamePiece.GamePiece;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.ArmSystem.Arm;
import frc.robot.subsystems.ArmSystem.BigArm;
import frc.robot.subsystems.ArmSystem.Claw;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  public final TankDrive mTankDrive = new TankDrive();
  public Arm littleArm = new Arm(Constants.getLittleArmConstants());
  public BigArm bigArm = new BigArm(Constants.getBigArmConstants());
  public Claw claw = new Claw();

  PneumaticHub pHub = new PneumaticHub();
  private final PDP pdp = new PDP();
  private final PersistenceData mData = new PersistenceData();
  private final ArmPositions armPositions = new ArmPositions(littleArm, bigArm);


  // public SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  private final CommandJoystick driveJoystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick buttonBox = new CommandJoystick(OperatorConstants.buttonBoxPort);
  private final CommandJoystick armJoystick = new CommandJoystick(OperatorConstants.armJoystickport);

  StayPutAllDOF stayPutCommand = new StayPutAllDOF(mTankDrive);
  CancelDriveTrain cancelCommand = new CancelDriveTrain(mTankDrive);

  AutoCommandFactory autoFactory;
  //private final TestCoolBeans t = new TestCoolBeans();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoFactory = new AutoCommandFactory(mTankDrive, littleArm, bigArm, claw);
    System.out.println("RUNNING ROBOT: " + Constants.uniqueRobotConstants.getName());
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

    driveJoystick.button(OperatorConstants.toggleTurning12).onTrue(Commands.runOnce(mTankDrive::toggleTurning, mTankDrive));
    //driveJoystick.button(OperatorConstants.cancelDrive11).onTrue(cancelCommand);
    driveJoystick.button(OperatorConstants.doAutoBalance10).whileTrue(new AutoBalancedPID(mTankDrive));
   // driveJoystick.button(12).whileTrue(getMiddleLeaveBalance());
   // driveJoystick.button(OperatorConstants.fullBalanceAct6).onTrue(driveToChargingStation.andThen(autoBalanceCommand));
    //driveJoystick.button(OperatorConstants.buttonUnused7).onTrue(new DriveToDistanceInches(mTankDrive, 10, .4));
    //driveJoystick.button(OperatorConstants.buttonUnused9).whileTrue(getMiddleLeaveBalance());
   // driveJoystick.button(OperatorConstants.buttonUnused7).onTrue(new DriveToObject(mTankDrive, ObjectType.CONE));
   // driveJoystick.button(OperatorConstants.buttonUnused9).onTrue(new DriveToObject(mTankDrive, ObjectType.CUBE));
    //driveJoystick.button(OperatorConstants.stayPut11).toggleOnTrue(stayPutCommand);

    //button box

    mTankDrive.setDefaultCommand( new TeleopDrive(mTankDrive, driveJoystick));
    driveJoystick.button(2).onTrue(Commands.runOnce(claw::toggle, claw));
    driveJoystick.button(3).whileTrue(new DriveToGamePiece(mTankDrive, GamePiece.both));


    armJoystick.button(5).onTrue(Commands.runOnce(littleArm::armConstantSpeedForwardFromDashBoard, littleArm)).onFalse(Commands.runOnce(littleArm::stop, littleArm));
    armJoystick.button(3).onTrue(Commands.runOnce(littleArm::armConstantSpeedBackwardFromDashBoard, littleArm)).onFalse(Commands.runOnce(littleArm::stop, littleArm));

    armJoystick.button(6).onTrue(Commands.runOnce(bigArm::armConstantSpeedForwardFromDashBoard, bigArm)).onFalse(Commands.runOnce(bigArm::stop, bigArm));
    armJoystick.button(4).onTrue(Commands.runOnce(bigArm::armConstantSpeedBackwardFromDashBoard, bigArm)).onFalse(Commands.runOnce(bigArm::stop, bigArm)); 

    armJoystick.button(10).whileTrue(new DriveToSubStationPickUp(mTankDrive));

    buttonBox.button(1).onTrue(armPositions.ArmsToSaftey());
    buttonBox.button(4).onTrue(armPositions.ArmsToHighConeDrop());
    buttonBox.button(3).onTrue(armPositions.ArmsToMidConeDrop());
    buttonBox.button(2).onTrue(armPositions.ArmsToFrontPickUp());
    buttonBox.button(8).onTrue(armPositions.ArmsToHighCubeDrop());
    buttonBox.button(7).onTrue(armPositions.ArmsToMidCubeDrop());
    buttonBox.button(6).onTrue(armPositions.ArmsToFrontPickUp());
    buttonBox.button(5).onTrue(armPositions.ArmsToFrontPickUp());
    buttonBox.button(12).onTrue(armPositions.ArmsToSubStationPickUp());
    buttonBox.button(13).onTrue(new CancelArms(littleArm, bigArm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoFactory.getAutoCommand();
  }

  public AutoCommandFactory getAutoCommandFactory() {
    return autoFactory;
  }

}