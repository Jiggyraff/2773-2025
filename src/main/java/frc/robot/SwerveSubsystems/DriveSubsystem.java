// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveSubsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Information.OdometrySubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveSubsystem extends SubsystemBase {
  public SwerveDriveModule blMotor = new SwerveDriveModule(Constants.backLeftModuleDriveCANID, Constants.backLeftModuleRotateCANID, Constants.backLeftModuleEncoderCANID, 0.3686);
  public SwerveDriveModule brMotor = new SwerveDriveModule(Constants.backRightModuleDriveCANID, Constants.backRightModuleRotateCANID, Constants.backRightModuleEncoderCANID, -0.1597);
  public SwerveDriveModule frMotor = new SwerveDriveModule(Constants.frontRightModuleDriveCANID, Constants.frontRightModuleRotateCANID, Constants.frontRightModuleEncoderCANID, -0.48657);
  public SwerveDriveModule flMotor = new SwerveDriveModule(Constants.frontLeftModuleDriveCANID, Constants.frontLeftModuleRotateCANID, Constants.frontLeftModuleEncoderCANID, 0.35522);
  public double setAngle;
  public double p = 0.63;
  public double i = 0;
  public double d = 0;
  public PIDController pid = new PIDController(p, i, d);

  
  public SwerveModulePosition[] getPositions()
  {
    return new SwerveModulePosition[] {
    flMotor.getSwervePosition(), frMotor.getSwervePosition(),
    blMotor.getSwervePosition(), brMotor.getSwervePosition()
  };
  }

  // PIDController flPID = new PIDController(0.63, 0, 0);
  // PIDController frPID = new PIDController(0.63, 0, 0);
  // PIDController blPID = new PIDController(0.63, 0, 0);
  // PIDController brPID = new PIDController(0.63, 0, 0);
  // boolean initDone = false;

  /** Creates a new TestSubsystem. */
  public DriveSubsystem() {
    Shuffleboard.getTab("Navigation").addDoubleArray("Set Angle gilcswdicqewe", () -> {return new double[] {setAngle};});
    
  }


  public SwerveDriveModule[] modules = {flMotor, blMotor, brMotor, frMotor};

  @Override
  public void periodic() {
      
  }

  public void drive(double speed, double rotate) {
    blMotor.drive(speed, rotate);
    brMotor.drive(speed, rotate);
    frMotor.drive(speed, rotate);
    flMotor.drive(speed, rotate);
  }

  public void directionalDrive(double speed, double angle) {
    blMotor.directionalDrive(speed, angle);
    brMotor.directionalDrive(speed, angle);
    frMotor.directionalDrive(speed, angle);
    flMotor.directionalDrive(speed, angle);
  }

  public void chassisDrive(ChassisSpeeds chassisSpeeds) {
    directionalDrive(
      Constants.powerVelocityRatio * Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2)),
      Constants.powerTwistRatio * Math.atan2(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond)
    );
  }

  static class Vec {
    double phi;
    double r;
    Vec(double r, double phi) {
      this.phi = phi; this.r = r;
    }
    Vec add(Vec a) {
      double x = this.r * Math.cos(this.phi);
      double y = this.r * Math.sin(this.phi);
      x += a.r * Math.cos(a.phi);
      y += a.r * Math.sin(a.phi);
      return new Vec(
        Math.sqrt(x * x + y * y),
        Math.atan2(y, x)
      );
    }
  }

  public void directionalDrive(double speed, double angle, double rotation) {
    Vec bl = new Vec(speed, angle).add(new Vec(rotation, -3 * Math.PI / 4));
    Vec br = new Vec(speed, angle).add(new Vec(rotation, 3 * Math.PI / 4));
    Vec fr = new Vec(speed, angle).add(new Vec(rotation, Math.PI / 4));
    Vec fl = new Vec(speed, angle).add(new Vec(rotation, -Math.PI / 4));
    blMotor.directionalDrive(bl.r, bl.phi);
    brMotor.directionalDrive(br.r, br.phi);
    frMotor.directionalDrive(fr.r, fr.phi);
    flMotor.directionalDrive(fl.r, fl.phi);
  }

  public void resetMotors() {
    blMotor.reset();
    brMotor.reset();
    frMotor.reset();
    flMotor.reset();
  }

  public void stop() {
    blMotor.stop();
    brMotor.stop();
    frMotor.stop();
    flMotor.stop();
  }

  public void rotate(double speed) {
    frMotor.directionalDrive(speed, Math.PI / 4);
    brMotor.directionalDrive(speed, 3 * Math.PI / 4);
    blMotor.directionalDrive(speed, -3 * Math.PI / 4);
    flMotor.directionalDrive(speed, -Math.PI / 4);
  }

  public void carDrive(double rotationFactor, double speed) {
    final double HALF_WHEEL_DISTANCE = Constants.DistanceBetweenWheels;
    double distance = 1 / (rotationFactor + 1e-7);

    speed *= Math.copySign(1, distance);

    double rl = Math.sqrt(
        HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE +
            (distance - HALF_WHEEL_DISTANCE) * (distance - HALF_WHEEL_DISTANCE));
    double rr = Math.sqrt(
        HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE +
            (distance + HALF_WHEEL_DISTANCE) * (distance + HALF_WHEEL_DISTANCE));

    double flAngle = Math.atan2(distance + HALF_WHEEL_DISTANCE, -HALF_WHEEL_DISTANCE);
    double frAngle = Math.atan2(distance - HALF_WHEEL_DISTANCE, -HALF_WHEEL_DISTANCE);
    double blAngle = Math.atan2(distance + HALF_WHEEL_DISTANCE, HALF_WHEEL_DISTANCE);
    double brAngle = Math.atan2(distance - HALF_WHEEL_DISTANCE, HALF_WHEEL_DISTANCE);

    double kl = 1, kr = 1;
    if (distance < 0) {
      kl = rr / rl;
    } else {
      kr = rl / rr;
    }

    frMotor.directionalDrive(kr*speed, frAngle);
    brMotor.directionalDrive(kr*speed, brAngle);
    blMotor.directionalDrive(kl*speed, blAngle);
    flMotor.directionalDrive(kl*speed, flAngle);
  }

  public void setPID(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;
    flMotor.setPIDValues(p, i, d);
    frMotor.setPIDValues(p, i, d);
    blMotor.setPIDValues(p, i, d);
    brMotor.setPIDValues(p, i, d);
  }

  public double getP() {
    return p;
  }

public PIDController getPID() {
    return pid;
}

  public double averageDistanceEncoder() {
    return (flMotor.distanceEncoderPosition()+frMotor.distanceEncoderPosition()+blMotor.distanceEncoderPosition()+brMotor.distanceEncoderPosition())/4;
  }

public void increasePBy(double e) {
    pid.setP(pid.getP() + e);
    System.out.println("PID set to:"+pid.getP());
}

public SwerveModuleState[] getStates() {
  return new SwerveModuleState[] {
    modules[0].getSwerveState(),
    modules[1].getSwerveState(),
    modules[2].getSwerveState(),
    modules[3].getSwerveState()
  };
}
}
