// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ArmCommands.ArmPositions;
import frc.robot.commands.AutoCommands.DriveToBalanced.DirectionB;
import frc.robot.commands.AutoCommands.DriveUntilUnBalanced.Direction;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.ArmSystem.Arm;
import frc.robot.subsystems.ArmSystem.BigArm;
import frc.robot.subsystems.ArmSystem.Claw;

/** Add your docs here. */
public class AutoCommandFactory {
    Arm littleArm;
    BigArm bigArm;
    TankDrive tankDrive;
    ArmPositions armPositions;
    Claw claw;

    public SendableChooser<Command> autoChooserStep1 = new SendableChooser<Command>();
    public SendableChooser<Command> autoChooserStep2 = new SendableChooser<Command>();

    SequentialCommandGroup testCommand;

    public AutoCommandFactory(TankDrive tankDrive, Arm littArm, BigArm bigArm, Claw claw){
        this.bigArm = bigArm;
        this.littleArm = littArm;
        this.tankDrive = tankDrive;
        this.claw = claw;
        this.armPositions = new ArmPositions(littArm, bigArm);

        testCommand = new SequentialCommandGroup();
       
        autoChooserStep1.setDefaultOption("No Drop Off", armPositions.ArmsToSaftey());
        autoChooserStep1.addOption("Mid Cube Drop Off", doDropOffAtMidCube());
        autoChooserStep1.addOption("High Cube Drop Off", doDropOffAtHighCube());
        autoChooserStep1.addOption("Mid Cone Drop Off", doDropOffAtMidCone());
        autoChooserStep1.addOption("High Cone Drop Off", doDropOffAtHighCone());
        autoChooserStep1.addOption("Low Drop Off", doDropOffAtLow());

        autoChooserStep2.setDefaultOption("Back Out Of Community", getLeaveAuto());
        autoChooserStep2.addOption("No Leave Balance", NoLeaveBalanceArmsToSaftey());
        autoChooserStep2.addOption("Leave and Balance", DriveOverChargeStationArmsToSaftey());
        autoChooserStep2.addOption("Don't Move", armPositions.ArmsToSaftey());

        SmartDashboard.putData(autoChooserStep2);
        SmartDashboard.putData(autoChooserStep1);
    }

    public SequentialCommandGroup doDropOffAtMidCube() {
        SequentialCommandGroup sCommandGroup = new SequentialCommandGroup();
        sCommandGroup.addCommands(armPositions.ArmsToMidCubeDrop());
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.3));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }

    public SequentialCommandGroup doDropOffAtHighCube() {
        SequentialCommandGroup sCommandGroup = new SequentialCommandGroup();
        sCommandGroup.addCommands(armPositions.ArmsToHighCubeDrop());
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.3));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }

    public SequentialCommandGroup doDropOffAtMidCone() {
        SequentialCommandGroup sCommandGroup=new SequentialCommandGroup();
        sCommandGroup.addCommands(armPositions.ArmsToMidConeDrop());
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.3));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }

    public SequentialCommandGroup doDropOffAtHighCone() {
        SequentialCommandGroup sCommandGroup=new SequentialCommandGroup();
        sCommandGroup.addCommands(armPositions.ArmsToHighConeDrop());
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubConeHighInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.3));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubConeHighInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }

    public SequentialCommandGroup doDropOffAtLow() {
        SequentialCommandGroup sCommandGroup=new SequentialCommandGroup();
        sCommandGroup.addCommands(armPositions.ArmsToFrontPickUp());
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.2));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }



    public SequentialCommandGroup getAutoCommand(){

        SequentialCommandGroup sCommandGroup = new SequentialCommandGroup();

        sCommandGroup.addCommands(this.AlwaysDo());
        sCommandGroup.addCommands(this.getStep1());
        sCommandGroup.addCommands(this.getStep2());

        return sCommandGroup;
    }

    private Command getStep1() {
        return autoChooserStep1.getSelected();
    }

    private Command getStep2() {
        return autoChooserStep2.getSelected();
    }

    public SequentialCommandGroup DriveOverChargeStation(){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new DriveUntilUnBalanced(tankDrive, Direction.backwards));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, AutoConstants.middleLeaveOnPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new DriveToBalanced(tankDrive, DirectionB.forwards));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, AutoConstants.middleLeaveOffPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new DriveUntilUnBalanced(tankDrive, Direction.forwards));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, 25, 0.6));
        commandGroup.addCommands(new AutoBalancedPID(tankDrive));
        return commandGroup;
    }

    public SequentialCommandGroup NoLeaveBalance(){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new DriveUntilUnBalanced(tankDrive, Direction.backwards));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, -30, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new AutoBalancedPID(tankDrive));
        return commandGroup;
    }

    public ParallelCommandGroup NoLeaveBalanceArmsToSaftey(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(NoLeaveBalance());
        pCommandGroup.addCommands(armPositions.ArmsToSaftey());
        return pCommandGroup;
    }


    public ParallelCommandGroup  DriveOverChargeStationArmsToSaftey(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(DriveOverChargeStation());
        pCommandGroup.addCommands(armPositions.ArmsToSaftey());
        return pCommandGroup;
    }

    public ParallelCommandGroup DriveOverChargeStationArmsFlipOver(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(DriveOverChargeStation());
        return pCommandGroup;
    }

    public SequentialCommandGroup getLeaveAuto() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(armPositions.ArmsToSaftey());
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, AutoConstants.leftLeaveDistanceInches, AutoConstants.leftLeaveSpeed));
        return commandGroup;
      }

    public SequentialCommandGroup AlwaysDo(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(Commands.runOnce(tankDrive.getGyro()::reset, tankDrive));
        pCommandGroup.addCommands(new EnableBrakes(tankDrive));
        pCommandGroup.addCommands(Commands.runOnce(claw::close, claw));
        pCommandGroup.addCommands(Commands.waitSeconds(.2));
        return pCommandGroup;
    }
}
