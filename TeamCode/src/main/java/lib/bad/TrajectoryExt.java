package lib.bad;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import lib.testingLib.geometry.Rotation2dExt;
import lib.testingLib.math.MathUtil;

public class TrajectoryExt {
    private final double m_totalTimeSeconds;
    private final List<StateExt> m_states;

    /**
     * Constructs a trajectory from a vector of states.
     *
     * @param states A vector of states.
     */
    public TrajectoryExt(List<StateExt> states) {
        this.m_states = states;
        this.m_totalTimeSeconds = m_states.get(m_states.size() - 1).timeSeconds;
    }

    public double getTotalTimeSeconds() {
        return m_totalTimeSeconds;
    }

    public List<StateExt> getStates() {
        return m_states;
    }

    public TrajectoryExt transformBy(Transform2d transform) {
        List<Trajectory.State> translationStates = new ArrayList<>();
        for (StateExt fullState : m_states) {
            translationStates.add(fullState.translationState);
        }

        List<Trajectory.State> transformedStates = new Trajectory(translationStates).transformBy(transform).getStates();

        List<StateExt> transformedFullStates = new ArrayList<>();
        for (int i = 0; i < m_states.size(); i++) {
            transformedFullStates.add(new StateExt(transformedStates.get(i), m_states.get(i).rotationState));
        }

        return new TrajectoryExt(transformedFullStates);
    }

    public TrajectoryExt rotateBy(Rotation2d rotate) {
        List<StateExt> rotatedStates = new ArrayList<>();
        for (StateExt state : this.m_states) {
            rotatedStates.add(new StateExt(
                    state.translationState,
                    new StateExt.RotationState(
                            state.timeSeconds,
                            state.rotationState.robotHeading.rotateBy(rotate),
                            state.rotationState.angularVelocityRadiansPerSecond,
                            state.rotationState.angularAccelerationRadiansPerSecondSq
                    )
            ));
        }

        return new TrajectoryExt(rotatedStates);
    }

    public Trajectory toTrajectory() {
        List<Trajectory.State> translationStates = new ArrayList<>();
        for (StateExt fullState : m_states) {
            translationStates.add(fullState.translationState);
        }

        return new Trajectory(translationStates);
    }

    public static class StateExt {
        public Trajectory.State translationState;
        public RotationState rotationState;
        public double timeSeconds;

        public StateExt() {
            translationState = new Trajectory.State();
            rotationState = new RotationState();
        }

        public StateExt(Trajectory.State translationState, RotationState rotationState) {
            if (Math.abs(translationState.timeSeconds - rotationState.timeSeconds) > 0.1) {
                throw new IllegalArgumentException("Both states must have the same time!");
            }

            this.translationState = translationState;
            this.rotationState = rotationState;
            this.timeSeconds = (translationState.timeSeconds + rotationState.timeSeconds) / 2;
        }

        StateExt interpolate(StateExt endValue, double i) {
            return new StateExt(
                    interpolateState(this.translationState, endValue.translationState, i),
                    this.rotationState.interpolate(endValue.rotationState, i)
            );
        }

        private Trajectory.State interpolateState(Trajectory.State startValue, Trajectory.State endValue, double i) {
            // Find the new t value.
            final double newT = MathUtil.interpolate(timeSeconds, endValue.timeSeconds, i);

            // Find the delta time between the current state and the interpolated state.
            final double deltaT = newT - timeSeconds;

            // If delta time is negative, flip the order of interpolation.
            if (deltaT < 0) {
                return interpolateState(endValue, startValue, 1 - i);
            }

            // Check whether the robot is reversing at this stage.
            final boolean reversing = startValue.velocityMetersPerSecond < 0
                    || Math.abs(startValue.velocityMetersPerSecond) < 1E-9 && startValue.accelerationMetersPerSecondSq < 0;

            // Calculate the new velocity
            // v_f = v_0 + at
            final double newV = startValue.velocityMetersPerSecond + (startValue.accelerationMetersPerSecondSq * deltaT);

            // Calculate the change in position.
            // delta_s = v_0 t + 0.5 at^2
            final double newS = (startValue.velocityMetersPerSecond * deltaT
                    + 0.5 * startValue.accelerationMetersPerSecondSq * Math.pow(deltaT, 2)) * (reversing ? -1.0 : 1.0);

            // Return the new state. To find the new position for the new state, we need
            // to interpolate between the two endpoint poses. The fraction for
            // interpolation is the change in position (delta s) divided by the total
            // distance between the two endpoints.
            final double interpolationFrac = newS
                    / endValue.poseMeters.getTranslation().getDistance(startValue.poseMeters.getTranslation());

            return new Trajectory.State(
                    newT, newV, startValue.accelerationMetersPerSecondSq,
                    startValue.poseMeters.plus((endValue.poseMeters.minus(startValue.poseMeters)).times(interpolationFrac)),
                    MathUtil.interpolate(startValue.curvatureRadPerMeter, endValue.curvatureRadPerMeter, interpolationFrac)
            );
        }

        public static class RotationState {
            public double timeSeconds;
            public Rotation2dExt robotHeading;
            public double angularVelocityRadiansPerSecond;
            public double angularAccelerationRadiansPerSecondSq;

            public RotationState() {
                this.robotHeading = new Rotation2dExt();
            }

            public RotationState(double timeSeconds, Rotation2d robotHeading, double angularVelocityRadiansPerSecond, double angularAccelerationRadiansPerSecondSq) {
                this.timeSeconds = timeSeconds;
                this.robotHeading = (Rotation2dExt) robotHeading;
                this.angularVelocityRadiansPerSecond = angularVelocityRadiansPerSecond;
                this.angularAccelerationRadiansPerSecondSq = angularAccelerationRadiansPerSecondSq;
            }

            RotationState interpolate(RotationState endValue, double i) {
                return new RotationState(
                        MathUtil.interpolate(this.timeSeconds, endValue.timeSeconds, i),
                        this.robotHeading.interpolate(endValue.robotHeading, i),
                        MathUtil.interpolate(this.angularVelocityRadiansPerSecond, endValue.angularVelocityRadiansPerSecond, i),
                        MathUtil.interpolate(this.angularAccelerationRadiansPerSecondSq, endValue.angularAccelerationRadiansPerSecondSq, i)
                );
            }

            @SuppressLint("DefaultLocale")
            @Override
            public String toString() {
                return String.format(
                        "RotationState(Sec. %.2f, Rotation r/s: %.2f, Heading: %s",
                        timeSeconds,
                        angularVelocityRadiansPerSecond,
                        robotHeading
                );
            }

            @Override
            public boolean equals(Object obj) {
                if (this == obj) {
                    return true;
                }
                if (!(obj instanceof RotationState)) {
                    return false;
                }
                RotationState state = (RotationState) obj;
                return Double.compare(timeSeconds, state.timeSeconds) == 0
                        && Double.compare(angularVelocityRadiansPerSecond, state.angularVelocityRadiansPerSecond) == 0
                        && Objects.equals(robotHeading, state.robotHeading);
            }

            @Override
            public int hashCode() {
                return Objects.hash(timeSeconds, angularVelocityRadiansPerSecond, robotHeading);
            }
        }
    }
}
