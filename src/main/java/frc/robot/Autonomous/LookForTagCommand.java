// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Information.TagSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LookForTagCommand extends Command {
  TagSubsystem tagSub;
  
  PIDController motorPID = new PIDController(0.15, 0, 0);
  /** Creates a new LockOnTagCommand. */
  public LookForTagCommand(TagSubsystem tagSub) {
    addRequirements(tagSub);
    // Use addRequirements() here to declare subsystem dependencies.
    this.tagSub = tagSub;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tagSub.reset();
    motorPID.setSetpoint(0);
    
  }
  
  int c = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (c == 2) {
      System.out.println("XCom: " + tagSub.getXCom() + "Sees Tag: " + tagSub.getSeestag());
      if (!tagSub.getSeestag()) {
        tagSub.setAngleDifference(0.05);
        // System.out.println(tagSub.getSeestag());
      } else {
        double speed = MathUtil.clamp(motorPID.calculate(-tagSub.getXCom()), -0.05, 0.05);
        tagSub.setAngleDifference(speed);
      }
      c = 0;
    } else {
      c++;
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
