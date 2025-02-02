// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Information.*;
import frc.robot.Constants;

public class HOTASDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final Joystick hotas;
  private final XboxController secondController;
  private final NavigationSubsystem navSub;
  private final TagSubsystem tagSub;
  private final OdometrySubsystem odomSub;

  private double setAngle;

  /** Creates a new DriveCommand. */
  public HOTASDriveCommand(DriveSubsystem driveSub, Joystick hotas, XboxController armStick, NavigationSubsystem navSub, TagSubsystem tagSub, OdometrySubsystem odomSub) {
    this.driveSubsystem = driveSub;
    this.hotas = hotas;
    this.secondController = armStick;
    this.navSub = navSub;
    this.tagSub = tagSub;
    this.odomSub = odomSub;
    addRequirements(driveSub);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setAngle = navSub.getGyroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    buttonMicroCommands();
    double x = hotas.getX(), y = hotas.getY();
    double angle = Math.atan2(y, x);
    double gyroAngle = navSub.getGyroAngle();
    double sensitivity = MathUtil.clamp(1 - hotas.getThrottle(), 0.05, 1);
    double speed = Math.sqrt(x * x + y * y) * Constants.DriveSpeedMultiplier * sensitivity;
    navSub.displayVariables(sensitivity);

    double r = (MathUtil.applyDeadband(hotas.getZ() + secondController.getLeftX(), Constants.HOTASRotationDeadzone)) * sensitivity;

    if (Math.abs(x) < Constants.HOTASDeadzone && Math.abs(y) < Constants.HOTASDeadzone && Math.abs(r)/sensitivity < Constants.HOTASRotationDeadzone) {
      driveSubsystem.stop();
    } else {
      if (Math.abs(r) > 0) {
        setAngle = navSub.getGyroAngle();
      }
      driveSubsystem.directionalDrive(speed, angle - gyroAngle, Constants.RotateSpeedMultiplier * r);
    }
        

  }

  public double calculateRotation() {
    PIDController rotatePID = new PIDController(0.1, 0, 0);
    rotatePID.setSetpoint(setAngle);
    // System.out.println(rotatePID.calculate(navSub.getGyroAngle()));
    return MathUtil.clamp(rotatePID.calculate(navSub.getGyroAngle())*100, -Constants.RotateSpeedMultiplier, Constants.RotateSpeedMultiplier);
  }

  public void buttonMicroCommands() {
    if (buttonPressed(7) && buttonOnPress(12)) {
      navSub.resetGyro();
      System.out.println("Gyro Reset Manually");
    }
    if (buttonPressed(7) && buttonOnPress(11)) {
      odomSub.setXY(0,0,navSub.getGyroAngle());
      System.out.println("Position Reset Manually");
    }
    if (buttonPressed(7) && buttonOnPress(9)) {
      tagSub.enable();
    }
    if (buttonPressed(7) && buttonOnPress(10)) {
      tagSub.cautiousMode();
    }
  }

  public Boolean buttonPressed(int i) {
    return hotas.getRawButton(i);
  }

  public Boolean buttonOnPress(int i) {
    return hotas.getRawButtonPressed(i);
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
