// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Information.LaserSubsystem;
import frc.robot.Information.TagSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LockOnTagCommand extends Command {
  TagSubsystem tagSub;
  LaserSubsystem laserSub;
  /** Creates a new LockOnTagCommand. */
  public LockOnTagCommand(TagSubsystem tagSub, LaserSubsystem laserSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tagSub = tagSub;
    this.laserSub = laserSub;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tagSub.reset();
    
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!tagSub.getSeestag()) {
      laserSub.setAngleDifference(0.05);
      System.out.println(tagSub.getSeestag());
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
