package frc.robot.Information;

import java.lang.reflect.Field;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveSubsystems.DriveSubsystem;
import frc.robot.SwerveSubsystems.SwerveDriveModule;

public class OdometrySubsystem extends SubsystemBase {

    DriveSubsystem driveSub;
    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(0.305, 0.305);
    Translation2d m_frontRightLocation = new Translation2d(0.305, -0.305);
    Translation2d m_backLeftLocation = new Translation2d(-0.305, 0.305);
    Translation2d m_backRightLocation = new Translation2d(-0.305, -0.305);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    SwerveDriveOdometry m_odometry;
    SwerveDriveModule[] modules;
    Pose2d pose = new Pose2d();
    Field2d field = new Field2d();

    public OdometrySubsystem(DriveSubsystem driveSub) {
        gyro.reset();
        this.driveSub = driveSub;
        modules = driveSub.modules;
        m_odometry = new SwerveDriveOdometry(
                m_kinematics, gyro.getRotation2d(),
                driveSub.getPositions(),
                new Pose2d(0.0, 0.0, new Rotation2d()));

        Shuffleboard.getTab("Odometry").addDouble("Robot X", () -> {return pose.getX();});
        Shuffleboard.getTab("Odometry").addDouble("Robot Y", () -> {return pose.getY();});
        SmartDashboard.putNumber("X", pose.getX());
        SmartDashboard.putNumber("Y", pose.getY());
    }
    
    // double oldY;
    // double oldX;
    @Override
    public void periodic() {
        Rotation2d gyroAngle = new Rotation2d(gyro.getAngle() * Math.PI / 180);
        pose = m_odometry.update(gyroAngle,
        driveSub.getPositions());
        // System.out.println(pose.getX() + ", " + pose.getY() + " " +
        field.setRobotPose(pose);
        SmartDashboard.putData("Field", field);
        // var dx = (getX() - oldX); var dy = (getY() - oldY);
        // var dc =Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        // System.out.println( dx + ", " + dy + " : " + dc);
        // oldY = getY();
        // oldX = getX();
    }

    public double getGyroAngle() {
        double angle = (gyro.getAngle() - 0) / 180.0 * Math.PI;
        while (angle > Math.PI) {
          angle -= Math.PI * 2;
        }
        while (angle < -Math.PI) {
          angle += Math.PI*2;
        }
        return angle;
      }

    public Pose2d getPose() {
        return pose;
    }
    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return pose.getY();
    }

    public void setXY(double x, double y, double rotation) {
        pose = new Pose2d(x, y, new Rotation2d(rotation));
        m_odometry.resetPose(pose);
    }

    public void resetGyro() {
        gyro.reset();
    }
}
