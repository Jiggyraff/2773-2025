// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveSubsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveSubsystem extends SubsystemBase {
  public SwerveDriveModule blMotor = new SwerveDriveModule(Constants.backLeftModuleDriveCANID, Constants.backLeftModuleRotateCANID, Constants.backLeftModuleEncoderCANID, 0.3686);
  public SwerveDriveModule brMotor = new SwerveDriveModule(Constants.backRightModuleDriveCANID, Constants.backRightModuleRotateCANID, Constants.backRightModuleEncoderCANID, -0.1597);
  public SwerveDriveModule frMotor = new SwerveDriveModule(Constants.frontRightModuleDriveCANID, Constants.frontRightModuleRotateCANID, Constants.frontRightModuleEncoderCANID, -0.48657);
  public SwerveDriveModule flMotor = new SwerveDriveModule(Constants.frontLeftModuleDriveCANID, Constants.frontLeftModuleRotateCANID, Constants.frontLeftModuleEncoderCANID, 0.35522);
  public double setAngle;

  
  // public SwerveModulePosition[] getPositions() {
  //   return new SwerveModulePosition[] {
  //   flMotor.getMotorEncoderPosition(), frMotor.getMotorEncoderPosition(),
  //   blMotor.getMotorEncoderPosition(), brMotor.getMotorEncoderPosition()
  //   };
  // }

  // PIDController flPID = new PIDController(0.63, 0, 0);
  // PIDController frPID = new PIDController(0.63, 0, 0);
  // PIDController blPID = new PIDController(0.63, 0, 0);
  // PIDController brPID = new PIDController(0.63, 0, 0);
  // boolean initDone = false;

  /** Creates a new TestSubsystem. */
  public DriveSubsystem() {
    // flPID.setSetpoint(0);
    // frPID.setSetpoint(0);
    // blPID.setSetpoint(0);
    // brPID.setSetpoint(0);
    Shuffleboard.getTab("Navigation").addDoubleArray("Set Angle gilcswdicqewe", () -> {
      return new double[] {setAngle};
    });
  }

  @Override
  public void periodic() {
    // if (flPID.atSetpoint() && frPID.atSetpoint() && blPID.atSetpoint() && brPID.atSetpoint()) {
    //   initDone = true;
    // }
    // if (!initDone) {
    //   flMotor.directionalDrive(0, flPID.calculate(flMotor.position()));
    //   frMotor.directionalDrive(0, frPID.calculate(frMotor.position()));
    //   blMotor.directionalDrive(0, blPID.calculate(blMotor.position()));
    //   brMotor.directionalDrive(0, brPID.calculate(brMotor.position()));
    // }
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

  public void rotateTo(double radians) {
    
  }

  public void setWheelStates(SwerveModuleState[] states) {
      flMotor.directionalDrive(states[0].speedMetersPerSecond, states[0].angle.getRadians());
      frMotor.directionalDrive(states[1].speedMetersPerSecond, states[1].angle.getRadians());
      blMotor.directionalDrive(states[2].speedMetersPerSecond, states[2].angle.getRadians());
      brMotor.directionalDrive(states[3].speedMetersPerSecond, states[3].angle.getRadians());
  }
  ;
}
