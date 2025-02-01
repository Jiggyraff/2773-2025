// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Information.NavigationSubsystem;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

public class PolarMoveCommand extends Command {

  double radians;
  double distance;
  DriveSubsystem driveSubsystem;
  double initDistance;
  final double tolerance = 0.1;     //Tolerance of fianl position coordinate in meters
  double speed;
  PIDController pid = new PIDController(0.63, 0, 0);
  OdometrySubsystem odomSub;
  

  /** Creates a new MovePolarCommand. */
  public PolarMoveCommand(double radians, double distance, DriveSubsystem driveSubsystem, OdometrySubsystem odomSub) {
    addRequirements(driveSubsystem);
    this.radians = radians;
    this.distance = distance;
    this.driveSubsystem = driveSubsystem;
    this.odomSub = odomSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(distance);
    pid.setTolerance(tolerance);
    initDistance = Math.sqrt(Math.pow(odomSub.getX(), 2)+Math.pow(odomSub.getY(), 2));
  }

  // Called every time the schetduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDistance = Math.sqrt(Math.pow(odomSub.getX(), 2)+Math.pow(odomSub.getY(), 2));
    double distanceDifference = Math.abs(currentDistance - initDistance);

    speed = MathUtil.clamp(pid.calculate(distanceDifference), -0.15, 0.15);
    System.out.println(distance + " , " + initDistance + " , " + currentDistance + " , " +distanceDifference);
    driveSubsystem.directionalDrive(speed, radians);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    System.out.println("Move Polar at Destination");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
