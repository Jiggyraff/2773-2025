// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OtherSubsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    Servo servo = new Servo(0);
    SparkMax motor = new SparkMax(13, Constants.motorType);
    RelativeEncoder encoder = motor.getEncoder();
    double speed;

    /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {
        Shuffleboard.getTab("Climber").addDouble("Encoder", () -> {
            return encoder.getPosition();
        });
        Shuffleboard.getTab("Climber").addDouble("Speed", () -> {
            return speed;
        });
        Shuffleboard.getTab("Climber").addDouble("Actual Speed", () -> {
            return encoder.getVelocity();
        });
    }

    @Override
    public void periodic() {

    }

    public void setSpeed(double s) {
        speed = s;
        motor.set(speed);
    }

    public void setServo(double value) {
        servo.set(value);
    }
}
