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
  private final LaserSubsystem laserSub;
  private final TagSubsystem tagSub;
  private final OdometrySubsystem odomSub;
  private PIDController pid;
  private PIDController rotatePID = new PIDController(0.63, 0, 0);

  private double setAngle;

  /** Creates a new DriveCommand. */
  public HOTASDriveCommand(DriveSubsystem driveSub, Joystick hotas, LaserSubsystem laserSub, TagSubsystem tagSub, OdometrySubsystem odomSub) {
    this.driveSubsystem = driveSub;
    this.hotas = hotas;
    this.laserSub = laserSub;
    this.tagSub = tagSub;
    this.odomSub = odomSub;
    this.pid = driveSub.getPID();
    addRequirements(driveSub);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setAngle = odomSub.getGyroAngle();
    odomSub.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  double oldT = 0.0;
  double oldG = 0.0;
  @Override
  public void execute() {
    buttonMicroCommands();
    double x = hotas.getX(), y = hotas.getY();
    double angle = Math.atan2(y, x);
    double gyroAngle = odomSub.getGyroAngle();
    double sensitivity = MathUtil.clamp(1 - hotas.getThrottle(), 0.05, 1);
    double setSpeed = MathUtil.clamp(Math.sqrt(x * x + y * y)*2, 0,2);
    double setRotation = (MathUtil.applyDeadband(hotas.getZ(), Constants.HOTASRotationDeadzone));
    // System.out.println(driveSubsystem.blMotor.distanceEncoderPosition() - oldT);
          
    pid.setSetpoint(setSpeed);
    double speed = pid.calculate((driveSubsystem.averageDistanceEncoder()-oldT)*11.24) * Constants.MaxDriveSpeed * sensitivity;
    pid.setSetpoint(setRotation);
    double rspeed = pid.calculate(((odomSub.getGyroAngle()-oldG))*2289);

    // System.out.println(speed);


    if (Math.abs(x) < Constants.HOTASDeadzone && Math.abs(y) < Constants.HOTASDeadzone && Math.abs(setRotation) < Constants.HOTASRotationDeadzone) {
      driveSubsystem.stop();
    } else {
      if (Math.abs(setRotation) > 0) {
        setAngle = odomSub.getGyroAngle();
      }
      driveSubsystem.directionalDrive(speed, angle - gyroAngle, setRotation * sensitivity * Constants.MaxRotationSpeed);
    }
    oldT = driveSubsystem.averageDistanceEncoder();
    oldG = odomSub.getGyroAngle();
  }

  // public double calculateRotation() {
  //   PIDController rotatePID = new PIDController(0.1, 0, 0);
  //   rotatePID.setSetpoint(setAngle);
  //   // System.out.println(rotatePID.calculate(odomSub.getGyroAngle()));
  //   return MathUtil.clamp(rotatePID.calculate(odomSub.getGyroAngle())*100, -Constants.RotateSpeedMultiplier, Constants.RotateSpeedMultiplier);
  // }

  public void buttonMicroCommands() {
    if (buttonPressed(7) && buttonOnPress(12)) {
      odomSub.resetGyro();
      System.out.println("Gyro Reset Manually");
    }
    if (buttonPressed(7) && buttonOnPress(11)) {
      odomSub.setPose(0,0,odomSub.getGyroAngle());
      System.out.println("Position Reset Manually");
    }
    if (buttonPressed(7) && buttonOnPress(9)) {
      tagSub.enable();
    }
    if (buttonPressed(7) && buttonOnPress(10)) {
      tagSub.cautiousMode();
    }
    if (buttonPressed(3)) {
      laserSub.setAngleDifference(0.05);
      // System.out.println("lk");
    }
    if (buttonPressed(4)) {
      laserSub.setAngleDifference(-0.05);
      // System.out.println("iiih");
    }
    if (buttonPressed(7) && buttonOnPress(8)) {
      laserSub.setEncoderZero();
      System.out.println("Laser position Reset Manually");
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
