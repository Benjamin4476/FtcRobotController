package org.firstinspires.ftc.teamcode.commands.drive;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.trajectory.constraint.MecanumDriveKinematicsConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Rotation2dExtended;

import java.util.ArrayList;
import java.util.List;

public class DriveAuto extends HolonomicControllerCommand {
    static PIDController xController = new PIDController(-7.1, 0.0, -0.2); //TODO: tune all of these
    static PIDController yController = new PIDController(7.1, 0.0, 0.2);
    static ProfiledPIDController thetaController = new ProfiledPIDController(-6.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(
                    Constants.DriveConstants.maxAttainableRotationRateRadiansPerSecond,
                    Constants.DriveConstants.maxAccelerationMetersPerSecondSquared));
    public static ElapsedTime time = new ElapsedTime();

    public DriveAuto(DriveSubsystem driveSubsystem, MecanumPath mecanumPath) {
        super(
                mecanumPath.trajectory,
                driveSubsystem::getOdometryLocation,
                xController, yController,
                thetaController,
                mecanumPath::sampleAngle,
                (speeds) -> driveSubsystem.robotDrive(speeds),
                driveSubsystem
        );
    }

    public static class MecanumPath {
        private final Pose2d startPose;
        private final List<Translation2d> points = new ArrayList<>();
        private final List<Rotation2d> angles = new ArrayList<>();
        private final List<Double> angleTimestamps = new ArrayList<>();
        Trajectory trajectory;

        public MecanumPath(double x, double y, double angle, double pathHeading) {
            this.startPose = new Pose2d(x, y, Rotation2d.fromDegrees(pathHeading));
            this.angles.add(Rotation2d.fromDegrees(angle));
        }

        public MecanumPath waypoint(double x, double y, double angle) {
            this.points.add(new Translation2d(x, y));
            this.angles.add(Rotation2d.fromDegrees(angle));

            return this;
        }

        @RequiresApi(api = Build.VERSION_CODES.N)
        public MecanumPath finish(double x, double y, double angle, double pathHeading, double maxSpeedM, MecanumDriveKinematics kinematics) {
            this.angles.add(Rotation2d.fromDegrees(angle));

            TrajectoryConfig config = new TrajectoryConfig(
                    maxSpeedM,
                    Constants.DriveConstants.maxAccelerationMetersPerSecondSquared);
            config.addConstraint(new MecanumDriveKinematicsConstraint(kinematics, maxSpeedM));

            this.trajectory = TrajectoryGenerator.generateTrajectory(startPose, points, new Pose2d(x, y, Rotation2d.fromDegrees(pathHeading)), config);

            for (Trajectory.State state : this.trajectory.getStates()) {
                if (this.points.contains(state.poseMeters.getTranslation())) {
                    this.angleTimestamps.add(state.timeSeconds);
                }
            }

            return this;
        }

        Rotation2d sampleAngle() {
            return new Rotation2d();
        }

        Rotation2d sampleAngle(double timeSeconds) {

            double totalTime = this.trajectory.getTotalTimeSeconds();
            int numStates = this.angles.size();

            if (timeSeconds <= 0) {
                return this.angles.get(0);
            }

            if (timeSeconds >= totalTime) {
                return this.angles.get(numStates - 1);
            }

            /*
            double progress = (timeSeconds / totalTime) * (numStates - 1);
            int index = (int) progress;
            double fractional = progress - (double) index;

            Rotation2dExtended startAngle = (Rotation2dExtended) this.angles.get(index);
            Rotation2d endAngle = this.angles.get(index + 1);
            return startAngle.interpolate(endAngle, fractional);*/

            int closest = -1;
            double closestSeparation = -1.0;
            for (int x = 0; x < angleTimestamps.size(); x++) {
                double separation = 0;
            }

            return new Rotation2d();
        }
    }
}
