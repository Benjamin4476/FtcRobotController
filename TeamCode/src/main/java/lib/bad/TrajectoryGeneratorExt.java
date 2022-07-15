package lib.bad;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import lib.testingLib.geometry.Rotation2dExt;

public final class TrajectoryGeneratorExt {
    private static final TrajectoryExt kDoNothingTrajectory = new TrajectoryExt(Collections.singletonList(new TrajectoryExt.StateExt()));

    private TrajectoryGeneratorExt() {}

    public static TrajectoryExt generateTrajectory(
            Pose2d start,
            Rotation2dExt pathStartHeading,
            List<Waypoint> interiorWaypoints,
            Pose2d end,
            Rotation2dExt pathEndHeading,
            TrajectoryConfigExt config
    ) {
        return kDoNothingTrajectory;
    }

    public static void generateTrajectory(
            Pose2d start,
            Rotation2d startHeading,
            @NonNull List<Waypoint> interiorWaypoints,
            Pose2d end,
            Rotation2d endHeading,
            TrajectoryConfigExt config
    ) {
        List<Translation2d> waypointLocations = new ArrayList<>();
        for (Waypoint waypoint : interiorWaypoints) {
            waypointLocations.add(waypoint.getLocation());
        }

        Trajectory locationTrajectory = TrajectoryGenerator.generateTrajectory(start, waypointLocations, end, config);
        TrajectoryExt fullTrajectory = generateTrajectoryWithHeadings(locationTrajectory, startHeading, interiorWaypoints, endHeading, config);

    }

    private static TrajectoryExt generateTrajectoryWithHeadings(
            Trajectory locationTrajectory,
            Rotation2d start,
            List<Waypoint> waypoints,
            Rotation2d end,
            TrajectoryConfigExt config
    ) {
        final double timeToMaxSpeed = config.getMaxAngularVelocity() / config.getMaxAngularAcceleration();
        final double distanceToMaxSpeed = (config.getMaxAngularAcceleration() * Math.pow(timeToMaxSpeed, 2)) / 2.0;

        // This is a highly inefficient way to do this.
        waypoints.add(0, new Waypoint(locationTrajectory.getInitialPose().getTranslation(), start));
        waypoints.add(new Waypoint(locationTrajectory.getStates().get(locationTrajectory.getStates().size() - 1).poseMeters.getTranslation(), end));

        List<Rotation2dExt> angles = new ArrayList<>();
        for (Waypoint waypoint : waypoints) angles.add((Rotation2dExt) waypoint.getHeading());

        List<TrajectoryExt.StateExt.RotationState> rotationStates = new ArrayList<>();

        // Generate the rotation states for the complete trajectory.
        // Iterate through the list of waypoints and generate states between waypoints
        for (int x = 0; x < angles.size() - 1; x++) {
            // Get the difference between the current waypoint angle and the next waypoint angle
            Rotation2d difference = angles.get(x).minus(angles.get(x + 1));
            double distance = difference.getRadians();
            boolean distanceIsNegative = distance < 0;
            distance = Math.abs(distance);

            // Because this system stops rotation at every waypoint, the rotation can be split in half for simplicity
            double semiDistance = distance / 2.0;

            // Calculate the minimum time to rotate from the current waypoint angle to the next
            double timeUnderAcceleration;
            double timeAtConstantSpeed;
            double semiTime;

            if (semiDistance > distanceToMaxSpeed) {
                timeUnderAcceleration = timeToMaxSpeed;
                timeAtConstantSpeed = (semiDistance - distanceToMaxSpeed) * config.getMaxAngularVelocity();
            } else {
                timeUnderAcceleration = Math.sqrt(2.0 * semiDistance / config.getMaxAngularAcceleration());
                timeAtConstantSpeed = 0;
            }

            semiDistance = (distanceIsNegative) ? -semiDistance : semiDistance;

            semiTime = timeUnderAcceleration + timeAtConstantSpeed;
            double minTime = semiTime * 2.0;

            // Get the trajectory's time to the next waypoint
            double trajectoryTimeToNextWaypoint;
            List<Trajectory.State> trajectoryStates = locationTrajectory.getStates();

            double startTime = 0;
            for (Trajectory.State state : trajectoryStates) {
                if (state.poseMeters.getTranslation().equals(waypoints.get(x).getLocation())) {
                    startTime = state.timeSeconds;
                    break;
                }
            }

            double endTime = 0;
            for (Trajectory.State state : trajectoryStates) {
                if (state.poseMeters.getTranslation().equals(waypoints.get(x + 1).getLocation())) {
                    endTime = state.timeSeconds;
                    break;
                }
            }

            trajectoryTimeToNextWaypoint = endTime - startTime;

            // If the minimum time to rotate is less than the trajectory's time,
            // generate rotation states to correspond with the location states
            if (minTime < trajectoryTimeToNextWaypoint) {
                for (int i = 0; i < locationTrajectory.getStates().size(); i++) {
                    // Get the location state to create a corresponding rotation state
                    Trajectory.State state = locationTrajectory.getStates().get(i);

                    // If the location state's time is greater than or equal to the end time,
                    // indicating that the next waypoint has been reached, exit this loop and proceed to the next
                    if (state.timeSeconds >= endTime) {
                        break;
                    }

                    // Store the previous rotation state for easy access. If this is the first time running this loop,
                    // there will not be a corresponding state, so a blank state must be created
                    TrajectoryExt.StateExt.RotationState prevState = (i == 0)
                            ? new TrajectoryExt.StateExt.RotationState(0, new Rotation2d(), 0, 0)
                            : rotationStates.get(i);


                    double halfTimeToNextWaypoint = trajectoryTimeToNextWaypoint / 2;
                    double acceleration = config.getMaxAngularAcceleration();
                    double velocity = 0;

                    double timeRemainingToHalfway = halfTimeToNextWaypoint - state.timeSeconds;
                    double distanceRemainingToHalfway = semiDistance - prevState.robotHeading.getRadians();

                    double distanceCoveredAtCurrentSpeed = rotationStates.get(i - 1).angularVelocityRadiansPerSecond * timeRemainingToHalfway;


                    if (rotationStates.get(i - 1).angularVelocityRadiansPerSecond * timeRemainingToHalfway >= distanceRemainingToHalfway){
                        continue;
                    }



                    if (i == 0) {
                        rotationStates.add(i, new TrajectoryExt.StateExt.RotationState(
                                state.timeSeconds,
                                start,
                                velocity,
                                acceleration
                        ));
                    }
                }
            }
        }

        /*
        rotationStates.add(
                new TrajectoryExt.StateExt.RotationState(
                        locationTrajectory.getTotalTimeSeconds(),
                        end,
                        0.0,
                        0.0
                )
        );*/

        List<TrajectoryExt.StateExt> fullTrajectoryStates = new ArrayList<>();
        for (int i = 0; i < locationTrajectory.getStates().size(); i++) {
            try {
                fullTrajectoryStates.add(new TrajectoryExt.StateExt(locationTrajectory.getStates().get(i), rotationStates.get(i)));
            } catch (IndexOutOfBoundsException e) {
                return kDoNothingTrajectory;
            }
        }

        return new TrajectoryExt(fullTrajectoryStates);
    }
}
