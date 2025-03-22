// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.Information.TagSubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToTagCommand extends Command {

  DriveSubsystem driveSub;
  OdometrySubsystem odomSub;
  TagSubsystem tagSub;

  double goalRotation;
  double distanceBack;

  final double speedTolerance = 0.1;     //Tolerance of fianl position coordinate in meters
  final double rotateTolerance = 5* Math.PI/180;     //Tolerance of fianl position coordinate in meters
  PIDController speedPID = new PIDController(0.63, 0, 0);
  PIDController rotationPID = new PIDController(0.3, 0, 0);

  /** Creates a new MoveToTagCommand. */
  public MoveToTagCommand(double distanceBack, DriveSubsystem driveSub, OdometrySubsystem odomSub, TagSubsystem tagSub) {
    addRequirements(driveSub, odomSub, tagSub);
    this.distanceBack = distanceBack;
    this.driveSub = driveSub;
    this.odomSub = odomSub;
    this.tagSub = tagSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (tagSub.getLastTagData() != null) {
    goalRotation = tagSub.aprilTagPositions[tagSub.getLastTagData().aprilTagID][4];
    
    speedPID.setSetpoint(0);
    speedPID.setTolerance(speedTolerance);
    rotationPID.setSetpoint(goalRotation);
    rotationPID.setTolerance(rotateTolerance);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tagSub.getLastTagData() != null) {
    double x = tagSub.getLastTagData().x;
    double z = tagSub.getLastTagData().z;

    double radians;
    double speed;

    double rspeed = MathUtil.clamp(rotationPID.calculate(odomSub.getGyroAngle()), -Constants.MaxRotationSpeed, Constants.MaxRotationSpeed);
    
    if (Math.abs(x) > 0.1) {
      //Look up truncated if statements
      radians = (x < 0) ? 0 : Math.PI;
      speed = MathUtil.clamp(speedPID.calculate(x),-Constants.MaxDriveSpeed,Constants.MaxDriveSpeed);
    } else {
      radians = Math.PI/2;
      speed = MathUtil.clamp(speedPID.calculate(z + distanceBack),-Constants.MaxDriveSpeed,Constants.MaxDriveSpeed);
    };

    driveSub.directionalDrive(speed, radians, rspeed);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (null != tagSub.getLastTagData() || (Math.abs(tagSub.getLastTagData().x) < 0.1 && tagSub.getLastTagData().z < distanceBack)) ? true : false;
  }
}
