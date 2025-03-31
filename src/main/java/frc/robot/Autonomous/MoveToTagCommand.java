// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
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

  XboxController xbox;

  double goalRotation;
  double distanceBack;

  final double speedTolerance = 0.06;     //Tolerance of fianl position coordinate in meters
  final double rotateTolerance = 5* Math.PI/180;     //Tolerance of fianl position coordinate in meters
  double xOffset;
  final double minDistanceBack = 0.11;
  PIDController speedPID = new PIDController(0.2, 0, 0);
  PIDController rotationPID = new PIDController(0.1, 0, 0);
  PIDController tinyPID = new PIDController(5, 0, 0);

  /** Creates a new MoveToTagCommand. */
  public MoveToTagCommand(double distanceBack, double xOffset,  DriveSubsystem driveSub, OdometrySubsystem odomSub, TagSubsystem tagSub, XboxController xbox) {
    addRequirements(driveSub, odomSub, tagSub);
    this.distanceBack = distanceBack;
    this.driveSub = driveSub;
    this.odomSub = odomSub;
    this.tagSub = tagSub;
    this.xOffset = xOffset;
    this.xbox = xbox;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (tagSub.getLastTagData() != null) {
    goalRotation = tagSub.aprilTagPositions[tagSub.getLastTagData().aprilTagID][4] * Math.PI / 180;
    System.out.println(goalRotation);
    speedPID.setSetpoint(0);
    speedPID.setTolerance(speedTolerance);
    rotationPID.setSetpoint(goalRotation);
    rotationPID.setTolerance(rotateTolerance);
    tinyPID.setSetpoint(0);
    tinyPID.setTolerance(0.02);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  double x, z, initX, initZ;
  boolean lastTag = true;
  @Override
  public void execute() {
    if (tagSub.getSeesTag()) {
      lastTag = true;
    }

    if (!tagSub.getSeesTag() && lastTag == true) {
      lastTag = false;
      System.out.println("SWAPPED TO ODOM");
      odomSub.setPose(0, 0, odomSub.getGyroAngle());
    }

    if (tagSub.getSeesTag()) {
      x = tagSub.getLastTagData().x + xOffset;
      z = tagSub.getLastTagData().z + distanceBack + minDistanceBack;
      initX = x;
      initZ = z;
    } else {
      // System.out.println("No Tag");
      x = initX + odomSub.getX();
      z = initZ + odomSub.getY();
    }
    

    double radians;
    double speed;

    double rspeed = MathUtil.clamp(rotationPID.calculate(odomSub.getGyroAngle()), -Constants.MaxRotationSpeed, Constants.MaxRotationSpeed);
    
    // if (Math.abs(x) > 0.1) {
    //   //Look up truncated if statements
    //   speed = MathUtil.clamp(speedPID.calculate(x),-Constants.MaxDriveSpeed,Constants.MaxDriveSpeed);
    //   if (x < 0) {
    //     radians = 0;
    //     speed *= -1;
    //   } else {
    //     radians = Math.PI;
    //   }
    // } else {
    //   radians = Math.atan2(-z, -x*1.5);
    //   speed = MathUtil.clamp(speedPID.calculate(z + distanceBack),-Constants.MaxDriveSpeed,Constants.MaxDriveSpeed);
    // };

    if (z < -0.1) {
      radians = Math.atan2(z, -x);
      speed = MathUtil.clamp(speedPID.calculate(Math.sqrt(Math.pow(z, 2) + Math.pow(x, 2))),-Constants.MaxDriveSpeed,Constants.MaxDriveSpeed);
      // System.out.println("X: " + x + " Z: " + z + " Speed: " + speed + " RSpeed: " + rspeed + " Radians: " + radians +" Gyro: " + odomSub.getGyroAngle());
    } else {
      radians = Math.atan2(z, -x);
      speed = MathUtil.clamp(tinyPID.calculate(Math.abs(x)), -Constants.MaxDriveSpeed, Constants.MaxDriveSpeed);
      // System.out.println("###X: " + x + " Z: " + z + " Speed: " + speed + " RSpeed: " + rspeed + " Radians: " + radians +" Gyro: " + odomSub.getGyroAngle());
    }


    driveSub.directionalDrive(-speed, radians, rspeed);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("At tag. Sees Tag: " + (tagSub.getSeesTag()) + " X: " + tagSub.getLastTagData().x + " Z: " + tagSub.getLastTagData().z);
    System.out.println(tagSub.getLastTagData().z > 0);
    System.out.println(distanceBack);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(x) < 0.02 && z > -0.05) || Math.abs(xbox.getLeftX()) > Constants.ControllerDeadzone) ? true : false;
  }
}
