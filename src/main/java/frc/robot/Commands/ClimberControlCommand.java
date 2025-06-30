package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
   

    double speed = Math.signum(MathUtil.applyDeadband(joy.getZ(), 0.5));
    climbSub.setSpeed(speed);

    
    climbSub.setServo(joy.getZ());
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
