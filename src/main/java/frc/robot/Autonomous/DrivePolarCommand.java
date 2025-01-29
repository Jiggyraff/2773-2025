// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Information.NavigationSubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

public class DrivePolarCommand extends Command {

  double radians;
  double distance;
  DriveSubsystem driveSubsystem;
  double initDistance;
  final double tolerance = 0.1;     //Tolerance of fianl position coordinate in meters
  double speed;
  PIDController pid = new PIDController(0.63, 0, 0);
  NavigationSubsystem navSub;
  

  /** Creates a new MovePolarCommand. */
  public DrivePolarCommand(double radians, double distance, DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    this.radians = radians;
    this.distance = distance;
    this.driveSubsystem = driveSubsystem;
    navSub = driveSubsystem.navSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(0);
    pid.setTolerance(tolerance);
    initDistance = navSub.averageDistanceTraveled();
  }

  // Called every time the schetduler runs while the command is scheduled.
  @Override
  public void execute() {
    double totalDistanceTraveled = navSub.averageDistanceTraveled();

    speed = MathUtil.clamp(-pid.calculate(distance), -0.3, 0.3);
    // System.out.println(distance + " , " + differenceX + " , " + differenceY);
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
