package lib.bad;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class Waypoint {
    private final Translation2d location;
    private final Rotation2d heading;

    public Waypoint(Translation2d location, Rotation2d heading) {
        this.location = location;
        this.heading = heading;
    }

    public Translation2d getLocation() {
        return location;
    }

    public Rotation2d getHeading() {
        return heading;
    }
}
