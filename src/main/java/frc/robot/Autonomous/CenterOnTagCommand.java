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
public class CenterOnTagCommand extends Command {

  DriveSubsystem driveSub;
  OdometrySubsystem odomSub;
  TagSubsystem tagSub;

  XboxController xbox;

  double goalRotation;

  final double speedTolerance = 0.06;     //Tolerance of fianl position coordinate in meters
  final double rotateTolerance = 5* Math.PI/180;     //Tolerance of fianl position coordinate in meters
  double xOffset;
  PIDController speedPID = new PIDController(0.2, 0, 0);
  PIDController rotationPID = new PIDController(0.1, 0, 0);
  PIDController tinyPID = new PIDController(5, 0, 0);

  /** Creates a new MoveToTagCommand. */
  public CenterOnTagCommand(double xOffset,  DriveSubsystem driveSub, OdometrySubsystem odomSub, TagSubsystem tagSub, XboxController xbox) {
    addRequirements(driveSub, odomSub, tagSub);
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
  double x, initX;
  boolean lastTag = true;
  @Override
  public void execute() {
    if (!tagSub.getSeesTag() && lastTag == true) {
      System.out.println("SWAPPED TO ODOM");
      odomSub.setPose(0, 0, odomSub.getGyroAngle());
    }
    lastTag = tagSub.getSeesTag();

    if (tagSub.getSeesTag()) {
      x = tagSub.getLastTagData().x + xOffset;
      initX = x;
    } else {
      // System.out.println("No Tag");
      x = initX + odomSub.getX();
    }

    double radians;
    double speed;
    double rspeed = MathUtil.clamp(rotationPID.calculate(odomSub.getGyroAngle()), -Constants.MaxRotationSpeed, Constants.MaxRotationSpeed);
    
    radians = Math.atan2(0, -x);
    if (Math.abs(x) > 0.1) {
      speed = MathUtil.clamp(speedPID.calculate(x, 2),-Constants.MaxDriveSpeed,Constants.MaxDriveSpeed);
    } else {
      speed = MathUtil.clamp(tinyPID.calculate(x, 2),-Constants.MaxDriveSpeed,Constants.MaxDriveSpeed);
    }

    driveSub.directionalDrive(speed, radians, rspeed);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("At tag. Sees Tag: " + (tagSub.getSeesTag()) + " X: " + tagSub.getLastTagData().x);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(x) < 0.02) || Math.abs(xbox.getLeftX()) > Constants.ControllerDeadzone || Math.abs(xbox.getLeftY()) > Constants.ControllerDeadzone) ? true : false;
  }
}
