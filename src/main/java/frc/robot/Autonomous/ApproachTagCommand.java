// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Information.TagSubsystem;
import frc.robot.Information.TagSubsystem.TagData;
import frc.robot.SwerveSubsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApproachTagCommand extends Command {
  TagSubsystem tagSub;
  DriveSubsystem driveSub;
  TagData data;
  /** Creates a new ApproachTagCommand. */
  public ApproachTagCommand(TagSubsystem tagSub, DriveSubsystem driveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tagSub = tagSub;
    this.driveSub = driveSub;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    data = tagSub.getLastTagData();
    // double angleToTag = Math.cosh(data.z/data.x);
    // double radius = data.z;
    // double newAngle = Math.abs(angleToTag)/angleToTag * Math.abs(angleToTag)-Math.PI/180;
    // double setX = -data.z
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (data.alpha > 0.1) {
      driveSub.directionalDrive(0.3, 0, 0);
    }
    if (data.alpha < -0.1) {
      driveSub.directionalDrive(0.3, Math.PI, 0);
    }
    if (Math.abs(data.alpha) < 0.1) {
      if (-data.z > 1) {
        driveSub.directionalDrive(0.3, Math.PI/2, 0);
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(data.alpha) < 0.1 && -data.z < 1) {
      System.out.println("Done");
      return true;
    } else return false;
  }
}
