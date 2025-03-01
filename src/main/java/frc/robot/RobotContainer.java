// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Autonomous.ApproachTagCommand;
import frc.robot.Autonomous.DeltaPoseCommand;
import frc.robot.Autonomous.LookForTagCommand;
import frc.robot.Autonomous.MoveElevatorCommand;
import frc.robot.Autonomous.PerfectPoseDifferenceCommand;
import frc.robot.Autonomous.PolarMoveCommand;
import frc.robot.Autonomous.RotateToRadiansCommand;
import frc.robot.Autonomous.RotateToCommand;
import frc.robot.Commands.*;
import frc.robot.Information.*;
import frc.robot.OtherSubsystems.ClimberSubsystem;
import frc.robot.OtherSubsystems.TowerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  // Auto chooser for Robot.java
  // Controllers
  Joystick hotaz = new Joystick(0);
  Joystick towerJoy = new Joystick(1);

  // Subsystems
  DriveSubsystem driveSub = new DriveSubsystem();
  OdometrySubsystem odomSub = new OdometrySubsystem(driveSub);
  TagSubsystem tagSub = new TagSubsystem(odomSub);
  LaserSubsystem laserSub = new LaserSubsystem(driveSub, tagSub, odomSub);
  TowerSubsystem towerSub = new TowerSubsystem();
  ClimberSubsystem climbSub = new ClimberSubsystem();
  
  // Commands from files
  HOTASDriveCommand driveCommand = new HOTASDriveCommand(driveSub, hotaz, laserSub, tagSub, odomSub);
  TowerControlCommand towerCommand = new TowerControlCommand(towerSub, towerJoy);
  ClimberControlCommand climberCommand = new ClimberControlCommand(climbSub, towerJoy);
  SimpleTowerControlCommand simpleTowerCommand = new SimpleTowerControlCommand(towerSub, towerJoy);
  LookForTagCommand lookForTagCommand = new LookForTagCommand(tagSub, laserSub);
  MoveElevatorCommand moveToMaxHeight = new MoveElevatorCommand(1, towerSub);
  {
    driveSub.setDefaultCommand(driveCommand);
    climbSub.setDefaultCommand(climberCommand);
    towerSub.setDefaultCommand(towerCommand);
    tagSub.setDefaultCommand(lookForTagCommand);
    NamedCommands.registerCommand("Elevator Max Height", moveToMaxHeight);
  }

  ApproachTagCommand tagCommand = new ApproachTagCommand(tagSub, driveSub);
  RotateToCommand rotateToCommand = new RotateToCommand(Math.PI, driveSub, odomSub);
  PolarMoveCommand polarMove = new PolarMoveCommand(-(3 / 4) * Math.PI, 1, driveSub, odomSub);
  RotateToRadiansCommand rotateToZero = new RotateToRadiansCommand(0, odomSub, driveSub);
  RotateToRadiansCommand rotateToFlank = new RotateToRadiansCommand(Math.PI, odomSub, driveSub);

  // SequentialCommandGroup plusPlus = new DeltaPoseCommand(0, 0.5, 0, driveSub, odomSub).andThen(
  //     new DeltaPoseCommand(0, -0.5, 0, driveSub, odomSub)).andThen(
  //         new DeltaPoseCommand(-0.5, 0, 0, driveSub, odomSub))
  //     .andThen(
  //         new DeltaPoseCommand(0.5, 0, 0, driveSub, odomSub))
  //     .andThen(
  //         new DeltaPoseCommand(0, -0.5, 0, driveSub, odomSub))
  //     .andThen(
  //         new DeltaPoseCommand(0, 0.5, 0, driveSub, odomSub))
  //     .andThen(
  //         new DeltaPoseCommand(0.5, 0, 0, driveSub, odomSub))
  //     .andThen(
  //         new DeltaPoseCommand(-0.5, 0, 0, driveSub, odomSub));

  // SequentialCommandGroup firstQuadrantPos = new DeltaPoseCommand(3, 0, Math.PI, driveSub, odomSub).andThen(
  //     new DeltaPoseCommand(0, -0.5, Math.PI, driveSub, odomSub).andThen(
  //         new DeltaPoseCommand(-2, 0, -Math.PI / 2, driveSub, odomSub)));

  // Buttons
  // driveStick
  // JoystickButton resetMotorsButton = new JoystickButton(driveStick, 4);
  // JoystickButton resetOrientationButton = new JoystickButton(hotas, 7);

  SequentialCommandGroup heightLadder = new MoveElevatorCommand(0.5, towerSub).andThen(
    new MoveElevatorCommand(0.25, towerSub).andThen(
      new MoveElevatorCommand(0.75, towerSub).andThen(
        new MoveElevatorCommand(0, towerSub)
      )
    )
  );

  private void configureBindings() {
    JoystickButton towerModeSwitchButton = new JoystickButton(towerJoy, 2);
    towerModeSwitchButton.toggleOnTrue(new InstantCommand(() -> {
      if (towerJoy.getRawButton(1)) {
        
      if (towerSub.getDefaultCommand() != simpleTowerCommand) {
        towerSub.setDefaultCommand(simpleTowerCommand);
        System.out.println("Simpletower");
      } else {
        towerSub.setDefaultCommand(towerCommand);
        System.out.println("Complextower");
      }
      towerSub.getDefaultCommand().schedule();
    }
    }));

  }

  SequentialCommandGroup doADance = new ParallelCommandGroup(
    new DeltaPoseCommand(0.5, 0.5, -Math.PI/2, driveSub, odomSub),
    new MoveElevatorCommand(0.25, towerSub)
  ).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(-1, 1, Math.PI/2, driveSub, odomSub),
    new MoveElevatorCommand(0.5, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(1, 1, -Math.PI/2, driveSub, odomSub),
    new MoveElevatorCommand(0.75, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(-1, 1, Math.PI/2, driveSub, odomSub),
    new MoveElevatorCommand(1, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(0.5, -3.5, 0, driveSub, odomSub),
    new MoveElevatorCommand(0, towerSub)
  ));

  public Command getAutonomousCommand(String autoChosen) {
    switch (autoChosen){
      case "Auto 1": return new PathPlannerAuto("Do A Flip");
      case "Auto 2": return new DeltaPoseCommand(0.5, 0.5, Math.PI/4, driveSub, odomSub);
      case "Auto 3": return new DeltaPoseCommand(1.5, 0.5, Math.PI/4, driveSub, odomSub);
      case "Auto 4": return new DeltaPoseCommand(-1.5, -0.5, 0, driveSub, odomSub);
      case "Auto 5": return doADance;
      case "Auto 6": return heightLadder;
            default: return null;
    }
  }
}
