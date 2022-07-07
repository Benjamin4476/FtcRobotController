package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;

public final class Constants {
    public static final class DriveConstants{
        public static final Translation2d frontLeftLocation = new Translation2d(0.167,0.195);
        public static final Translation2d frontRightLocation = new Translation2d(0.167,-0.195);
        public static final Translation2d backLeftLocation = new Translation2d(-0.167,0.195);
        public static final Translation2d backRightLocation = new Translation2d(-0.167,-0.195);

        public static final double CPR = 537.6; // Encoder counts per wheel rotation
        public static final double WHEEL_DIAMETER = 0.1016; // Wheel diameter in meters
        public static final double TICKS_TO_METERS = (WHEEL_DIAMETER * Math.PI / CPR); // Wheel distance traveled per encoder tick in meters
        public static final double METERS_TO_TICKS = 1 / TICKS_TO_METERS; // Convert meters per second to ticks per second

        public static final double maxAttainableSpeedMetersPerSecond = 4.1; // TODO: set these to correct values
        public static final double maxAttainableRotationRateRadiansPerSecond = 8.0;
        public static final double maxAccelerationMetersPerSecondSquared = 2.9;
    }
}
