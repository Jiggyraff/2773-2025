// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveSubsystems.*;
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
  public Command getAutonomousCommand() {
    return null;
  }
  // Controllers
  XboxController driveStick = new XboxController(0);
  XboxController armStick = new XboxController(1);

  // Subsystems
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  NavigationSubsystem navigationSubsystem = new NavigationSubsystem();
  KinematicsSubsystem kinematicsSubsystem = new KinematicsSubsystem(driveSubsystem);

  // Commands from files
    //Drive Commands
  DriveCommand driveCommand = new DriveCommand(driveSubsystem, driveStick, kinematicsSubsystem);

  //Buttons
    //driveStick
  JoystickButton turnToTagButton = new JoystickButton(driveStick, 2);
  // JoystickButton resetMotorsButton = new JoystickButton(driveStick, 4);
  JoystickButton resetOrientationButton = new JoystickButton(driveStick, 7);
    
  //armStick
    JoystickButton intakeButton = new JoystickButton(armStick, 2);
    JoystickButton shootButton = new JoystickButton(armStick, 6);

  JoystickButton sideSpeakerShootButton = new JoystickButton(armStick, 4);
  JoystickButton middleSpeakerShootButton = new JoystickButton(armStick, 3);
  JoystickButton reverseIntakeButton = new JoystickButton(armStick, 1);
  JoystickButton reverseShooterButton = new JoystickButton(armStick, 5);
    POVButton dpadDownButton = new POVButton(armStick, 0);
    POVButton dpadRightButton = new POVButton(armStick, 90);
    POVButton dpadLeftButton = new POVButton(armStick, -90);
    POVButton dpadUpButton = new POVButton(armStick, 180);
  
  private void configureBindings() {
    driveSubsystem.setDefaultCommand(driveCommand);
  }
}
