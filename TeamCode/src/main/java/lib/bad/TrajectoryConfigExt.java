package lib.bad;

import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;

public class TrajectoryConfigExt extends TrajectoryConfig {
    private final double maxAngularVelocityRadiansPerSecond;
    private final double maxAngularAccelerationRadiansPerSecondSq;

    /**
     * Constructs the trajectory configuration class.
     *
     * @param maxVelocityMetersPerSecond       The max velocity for the trajectory.
     * @param maxAccelerationMetersPerSecondSq The max acceleration for the trajectory.
     */
    public TrajectoryConfigExt(
            double maxVelocityMetersPerSecond,
            double maxAccelerationMetersPerSecondSq,
            double maxAngularVelocityRadiansPerSecond,
            double maxAngularAccelerationRadiansPerSecondSq
    ) {
        super(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);
        this.maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
        this.maxAngularAccelerationRadiansPerSecondSq = maxAngularAccelerationRadiansPerSecondSq;
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocityRadiansPerSecond;
    }

    public double getMaxAngularAcceleration() {
        return maxAngularAccelerationRadiansPerSecondSq;
    }
}
