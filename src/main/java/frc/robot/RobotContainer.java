// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveSubsystems.*;
import frc.SamSpaghettiCode.TowerSubsystem;
import frc.robot.Autonomous.ApproachTagCommand;
import frc.robot.Autonomous.DeltaPoseCommand;
import frc.robot.Autonomous.LockOnTagCommand;
import frc.robot.Autonomous.PolarMoveCommand;
import frc.robot.Autonomous.RotateToRadiansCommand;
import frc.robot.Autonomous.RotateToCommand;
import frc.robot.Commands.*;
import frc.robot.Information.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }
  //Auto chooser for Robot.java
  // Controllers
  Joystick hotaz = new Joystick(0);
  
  // XboxController secondController = new XboxController(1);
  
  // Subsystems
  DriveSubsystem driveSub = new DriveSubsystem();
  OdometrySubsystem odomSub = new OdometrySubsystem(driveSub);
  TagSubsystem tagSub = new TagSubsystem(odomSub);
  LaserSubsystem laserSub = new LaserSubsystem(driveSub, tagSub, odomSub);
  TowerSubsystem towerSub = new TowerSubsystem();
  
  // Commands from files
  HOTASDriveCommand driveCommand = new HOTASDriveCommand(driveSub, hotaz, laserSub, tagSub, odomSub);

  
  ApproachTagCommand tagCommand = new ApproachTagCommand(tagSub, driveSub);
  RotateToCommand rotateToCommand = new RotateToCommand(Math.PI, driveSub, odomSub);
  PolarMoveCommand polarMove = new PolarMoveCommand(-(3/4)*Math.PI, 1, driveSub, odomSub);
  RotateToRadiansCommand rotateToZero = new RotateToRadiansCommand(0, odomSub, driveSub);
  RotateToRadiansCommand rotateToFlank = new RotateToRadiansCommand(Math.PI, odomSub, driveSub);
  SequentialCommandGroup triangle = 
    new PolarMoveCommand(-Math.PI/4, 1, driveSub, odomSub
  ).andThen(
    new PolarMoveCommand(-(3/4)*Math.PI, 1, driveSub, odomSub)
  ).andThen(
    new PolarMoveCommand((1/2)*Math.PI, 1, driveSub, odomSub)
  );
  SequentialCommandGroup ladder = 
    new PolarMoveCommand(0, 1, driveSub, odomSub
  ).andThen(
    new PolarMoveCommand(-Math.PI/4, 1, driveSub, odomSub
  ).andThen(
    new PolarMoveCommand(-Math.PI/2, 1, driveSub, odomSub)
  ).andThen(
    new PolarMoveCommand(3*-Math.PI/4, 1, driveSub, odomSub)
  ).andThen(
    new PolarMoveCommand(-Math.PI, 1, driveSub, odomSub)
  ));
  SequentialCommandGroup plus = 
    new PolarMoveCommand(0, 0.5, driveSub, odomSub
  ).andThen(
    new PolarMoveCommand(Math.PI, 0.5, driveSub, odomSub)
  ).andThen(
    new PolarMoveCommand(Math.PI/2, 0.5, driveSub, odomSub)
  ).andThen(
    new PolarMoveCommand(3*Math.PI/2, 0.5, driveSub, odomSub)
  ).andThen(
    new PolarMoveCommand(Math.PI, 0.5, driveSub, odomSub)
  ).andThen(
    new PolarMoveCommand(0, 0.5, driveSub, odomSub)
  ).andThen(
    new PolarMoveCommand(3*Math.PI/2, 0.5, driveSub, odomSub)
  ).andThen(
    new PolarMoveCommand(Math.PI/2, 0.5, driveSub, odomSub)
  );

  SequentialCommandGroup plusPlus = 
    new DeltaPoseCommand(0, 0.5, 0, driveSub, odomSub
  ).andThen(
    new DeltaPoseCommand(0, -0.5, 0, driveSub, odomSub)
  ).andThen(
    new DeltaPoseCommand(-0.5, 0, 0, driveSub, odomSub)
  ).andThen(
    new DeltaPoseCommand(0.5, 0, 0, driveSub, odomSub)
  ).andThen(
    new DeltaPoseCommand(0, -0.5, 0, driveSub, odomSub)
  ).andThen(
    new DeltaPoseCommand(0, 0.5, 0, driveSub, odomSub)
  ).andThen(
    new DeltaPoseCommand(0.5, 0, 0, driveSub, odomSub)
  ).andThen(
    new DeltaPoseCommand(-0.5, 0, 0, driveSub, odomSub)
  );

  SequentialCommandGroup firstQuadrantPos = new DeltaPoseCommand(3, 0, Math.PI, driveSub, odomSub).andThen(
    new DeltaPoseCommand(0, -0.5, Math.PI, driveSub, odomSub).andThen(
      new DeltaPoseCommand(-2, 0, -Math.PI/2, driveSub, odomSub)
    )
  );

  SequentialCommandGroup findAndApproachTag = new LockOnTagCommand(tagSub, laserSub).andThen();
  //Buttons
  //driveStick
  // JoystickButton resetMotorsButton = new JoystickButton(driveStick, 4);
  // JoystickButton resetOrientationButton = new JoystickButton(hotas, 7);
  
  
    
  
  private void configureBindings() {
    driveSub.setDefaultCommand(driveCommand);
  }
  public Command getAutonomousCommand() {
    return findAndApproachTag;
  }
}
