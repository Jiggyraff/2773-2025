// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Information.*;
import frc.robot.Constants;

public class AlternativeDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final XboxController joy;
  private final NavigationSubsystem navigationSubsystem;

  /** Creates a new DriveCommand. */
  public AlternativeDriveCommand(DriveSubsystem driveSubsystem, XboxController joy, NavigationSubsystem navigationSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.joy = joy;
    this.navigationSubsystem = navigationSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  private final Translation2d[] modulePos = {
  new Translation2d(-Constants.DistanceBetweenWheels/2, Constants.DistanceBetweenWheels/2),
  new Translation2d(Constants.DistanceBetweenWheels/2, Constants.DistanceBetweenWheels/2),
  new Translation2d(-Constants.DistanceBetweenWheels/2, -Constants.DistanceBetweenWheels/2),
  new Translation2d(Constants.DistanceBetweenWheels/2, -Constants.DistanceBetweenWheels/2),
  };

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double Deadzone = Constants.ControllerDeadzone;
    double x = joy.getLeftX(), y = joy.getLeftY(), r = joy.getRightX();
    double angle = Math.atan2(y, x);
    double gyroAngle = navigationSubsystem.angle();
 
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modulePos);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, r), new Translation2d());
    kinematics.desaturateWheelSpeeds(states, Constants.maxWheelSpeed);

    if (Math.abs(x) < Deadzone && Math.abs(y) < Deadzone && Math.abs(r) < Deadzone) {
      driveSubsystem.stop();
    } else {
      driveSubsystem.setWheelStates(states);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
