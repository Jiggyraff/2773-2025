package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OtherSubsystems.ClimberSubsystem;

public class ClimberControlCommand extends Command {

  ClimberSubsystem climbSub;
  Joystick joy;

  public ClimberControlCommand(ClimberSubsystem climbSub, Joystick joy) {
    addRequirements(climbSub);
    this.climbSub = climbSub;
    this.joy = joy;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (joy.getRawButton(7)) {
      climbSub.setSpeed(-0.15);
    }
    else if (joy.getRawButton(6)) {
      climbSub.setSpeed(0.15);
    }
    else {
      climbSub.setSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
