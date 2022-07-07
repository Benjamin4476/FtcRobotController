package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;


public class DriveSubsystem extends SubsystemBase {
    private final ElapsedTime runtime = new ElapsedTime();

    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx backLeftMotor;
    private final MotorEx backRightMotor;

    private final GyroEx gyro;

    public final MecanumDriveKinematics kinematics;
    private final MecanumDriveOdometry odometry;

    private final Telemetry telemetry;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        gyro = new RevIMU(hardwareMap, "imu");

        kinematics = new MecanumDriveKinematics(
                Constants.DriveConstants.frontLeftLocation,
                Constants.DriveConstants.frontRightLocation,
                Constants.DriveConstants.backLeftLocation,
                Constants.DriveConstants.backRightLocation
        );

        odometry = new MecanumDriveOdometry(kinematics, Rotation2d.fromDegrees(0));

        gyro.init();

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        odometry.updateWithTime(
                runtime.time(),
                gyro.getRotation2d(),
                new MecanumDriveWheelSpeeds(
                        frontLeftMotor.getVelocity() / Constants.DriveConstants.METERS_TO_TICKS,
                        frontRightMotor.getVelocity() / Constants.DriveConstants.METERS_TO_TICKS,
                        backLeftMotor.getVelocity() / Constants.DriveConstants.METERS_TO_TICKS,
                        backRightMotor.getVelocity() / Constants.DriveConstants.METERS_TO_TICKS
                )
        );

        telemetry.addData("Odometry Location", getOdometryLocation());
    }

    public void robotDrive(double forward, double right, double rotation, boolean fieldCentric) {
        ChassisSpeeds chassisSpeeds;

        if (fieldCentric) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, right, rotation, odometry.getPoseMeters().getRotation());
        } else {
            chassisSpeeds = new ChassisSpeeds(forward, right, rotation);
        }

        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        wheelSpeeds.normalize(0);
        setMotors(wheelSpeeds);
    }

    public void robotDrive(ChassisSpeeds speeds) {
        setMotors(kinematics.toWheelSpeeds(speeds));
    }

    public Pose2d getOdometryLocation() {
        return odometry.getPoseMeters();
    }

    public void setMotors(MecanumDriveWheelSpeeds speeds) {
        frontLeftMotor.setVelocity(speeds.frontLeftMetersPerSecond * Constants.DriveConstants.METERS_TO_TICKS);
        frontRightMotor.setVelocity(speeds.frontRightMetersPerSecond * Constants.DriveConstants.METERS_TO_TICKS);
        backLeftMotor.setVelocity(speeds.rearLeftMetersPerSecond * Constants.DriveConstants.METERS_TO_TICKS);
        backRightMotor.setVelocity(speeds.rearRightMetersPerSecond * Constants.DriveConstants.METERS_TO_TICKS);
    }

    public void stop() {
        frontLeftMotor.stopMotor();
        frontRightMotor.stopMotor();
        backLeftMotor.stopMotor();
        backRightMotor.stopMotor();
    }
}
