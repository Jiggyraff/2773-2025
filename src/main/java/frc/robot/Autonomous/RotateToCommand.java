// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveSubsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToCommand extends Command {
  DriveSubsystem driveSub;
  double angle;
  double speed;
  /** Creates a new RotateToCommand. */
  public RotateToCommand(DriveSubsystem driveSub, double angle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSub = driveSub;
    this.angle = angle;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (Math.abs(driveSub.gyroAngle - angle) > Math.PI) {
      driveSub.directionalDrive(0, 0, 0.3);
    // } else {
    //   driveSub.directionalDrive(0, 0, -0.3);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(angle + ", " + driveSub.gyroAngle);
    if ((angle - 0.05) < driveSub.gyroAngle && driveSub.gyroAngle < (angle + 0.05)) {
      return true;
    } else {
      return false;
    }
  }
}
