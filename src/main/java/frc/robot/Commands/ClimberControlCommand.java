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
      // System.out.println("Climber at negative speed");
    } else if (joy.getRawButton(6)) {
      climbSub.setSpeed(0.15);
      // System.out.println("Climber at positive speed");
    } else {
      climbSub.setSpeed(0);
    }

    if (joy.getRawButton(4) && joy.getRawButtonPressed(1)) {
      climbSub.setServo(1);
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
