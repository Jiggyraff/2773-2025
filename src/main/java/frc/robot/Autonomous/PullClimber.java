// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.OtherSubsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PullClimber extends Command {

  ClimberSubsystem climbSub;
  double s;

  public PullClimber(ClimberSubsystem climbSub, double s) {
    addRequirements(climbSub);
    this.climbSub = climbSub;
    this.s = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSub.setSpeed(s);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
