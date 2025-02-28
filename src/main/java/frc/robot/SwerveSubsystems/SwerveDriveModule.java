// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveSubsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class SwerveDriveModule {
  private static final double ROTATION_LIMIT_SPEED = 0.7;

  public SparkMax driveMotor;
  public SparkMax rotateMotor;
  public CANcoder encoder;
  public RelativeEncoder distanceEncoder;
  public int id;
  public double alpha;
  private PIDController pidRotate;

  private static double DriveMotorWheelGearRatio = 1.0 / 6.75;
  private static double EncoderMagicRevolutionNumber = 0.047964; //  42/1024 = resolution/1024
  
  private double oldDistance = 0.0;
  private double velocity = 0.0;

  public SwerveDriveModule(int driveId, int rotateId, int encoderId, double alpha) {
    driveMotor = new SparkMax(driveId, frc.robot.Constants.motorType);
    rotateMotor = new SparkMax(rotateId, frc.robot.Constants.motorType);
    encoder = new CANcoder(encoderId);
    distanceEncoder = driveMotor.getEncoder();
    id = encoderId;
    this.alpha = alpha;
    this.pidRotate = new PIDController(0.30, 0, 0);
    distanceEncoder.setPosition(0);
  }

  public double distanceEncoderPosition() {
    return distanceEncoder.getPosition() / EncoderMagicRevolutionNumber * DriveMotorWheelGearRatio
        * Constants.WheelCircumference;
  }

  public void drive(double speed, double rotate) {
    driveMotor.set(speed);
    rotateMotor.set(rotate);
  }

  public void directionalDrive(double speed, double angle) {
    pidRotate.setSetpoint(0);
    double pos = canCoderPositionAdjusted() - angle;
    while (pos < -Math.PI)
      pos += 2 * Math.PI;
    while (pos >= Math.PI)
      pos -= 2 * Math.PI;
    // -PI =< pos < PI

    double direction = 1.0;
    if (pos < -Math.PI / 2) {
      direction = -1.0;
      pos += Math.PI;
    }
    if (pos > Math.PI / 2) {
      direction = -1.0;
      pos -= Math.PI;
    }
    double speedOfRotation = pidRotate.calculate(pos);
    speedOfRotation = MathUtil.clamp(speedOfRotation, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
    rotateMotor.set(speedOfRotation);

    driveMotor.set(speed * direction);
  }

  public double canCoderPositionAdjusted() {
    // position [-0.5..0.5)
    double value = encoder.getAbsolutePosition().getValueAsDouble() - alpha;
    if (value < -0.5)
      value += 1.0;
    if (value >= 0.5)
      value -= 1.0;
    return -value * 2 * Math.PI;
  }

  public void reset() {
    pidRotate.setSetpoint(0);
    double s = pidRotate.calculate(canCoderPositionAdjusted());
    s = MathUtil.clamp(s, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
    rotateMotor.set(s);
  }

  public void stop() {
    driveMotor.stopMotor();
    rotateMotor.stopMotor();
  }

  public void setPIDValues(double p, double i, double d) {
    pidRotate.setPID(p, i, d);
  }

  public SwerveModulePosition getSwervePosition() {
    return new SwerveModulePosition(
        distanceEncoderPosition(), new Rotation2d(canCoderPositionAdjusted()));
  }

  public SwerveModuleState getSwerveState() {
    return new SwerveModuleState(
        distanceEncoder.getVelocity(), new Rotation2d(canCoderPositionAdjusted()));
  }
}
