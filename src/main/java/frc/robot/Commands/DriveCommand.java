// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Information.*;
import frc.robot.Constants;

public class DriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final Joystick hotas;
  private final XboxController secondController;
  private final NavigationSubsystem navigationSubsystem;

  private double lastAngle = 0.0;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, Joystick hotas, XboxController armStick, NavigationSubsystem navigationSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.hotas = hotas;
    this.secondController = armStick;
    this.navigationSubsystem = navigationSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = hotas.getX(), y = hotas.getY();
    double angle = Math.atan2(y, x);
    double gyroAngle = navigationSubsystem.getAdjustedAngle();
    // double sensitivity = MathUtil.clamp(1 - hotas.getThrottle(), 0.05, 1);
    double speed = Math.sqrt(x * x + y * y) * Constants.DriveSpeedMultiplier;

    /* if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
       double rotate = joy.getRightX();
       if (Math.abs(rotate) > Deadzone) {
         driveSubsystem.rotate(Constants.RotateSpeedMultiplier * rotate);
       } else {
         driveSubsystem.stop();
       }
     } else {
       driveSubsystem.directionalDrive(speed, angle - gyroAngle);
     }*/
    
    double r = hotas.getZ() + secondController.getLeftX();// + (Math.abs(driveSubsystem.setAngle) - Math.abs(gyroAngle));

    r = MathUtil.applyDeadband(r, Constants.HOTASRotationDeadzone);

    if (Math.abs(x) < Constants.HOTASDeadzone && Math.abs(y) < Constants.HOTASDeadzone && Math.abs(r) < Constants.HOTASRotationDeadzone) {
      driveSubsystem.stop();
    } else {
      driveSubsystem.directionalDrive(speed, angle - gyroAngle, Constants.RotateSpeedMultiplier * r);
      lastAngle = gyroAngle;
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
