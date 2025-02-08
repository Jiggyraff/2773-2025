// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.SwerveSubsystems.*;

public class RotateToCommand extends Command {

  DriveSubsystem driveSub;
  OdometrySubsystem odomSub;
  double destination;

  PIDController rotatePID = new PIDController(0.63, 0, 0);

  public RotateToCommand(double radians, DriveSubsystem driveSubsystem, OdometrySubsystem navSub) {
    addRequirements(driveSubsystem);
    this.driveSub = driveSubsystem;
    this.odomSub = navSub;
    this.destination = destination;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double radians = destination - odomSub.getGyroAngle();
    rotatePID.setSetpoint(radians);
    rotatePID.setTolerance(0.01);
    System.out.println();
    System.out.println("Rotate Robot Going To: " + destination + " , "  + (destination - odomSub.getGyroAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedOfRotation = rotatePID.calculate(odomSub.getGyroAngle());
    speedOfRotation = MathUtil.clamp(speedOfRotation, -0.3, 0.3);
    driveSub.rotate(speedOfRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stop();
    System.out.println("Rotate Robot Ended At:" + odomSub.getGyroAngle());
    System.out.println();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePID.atSetpoint();
  }
}
