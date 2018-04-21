package org.hmorgan.deadreckoning;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

/**
 * Simple data structure representing an entity's state in ECEF frame
 *
 * @author Hunter N. Morgan
 */
public class EntityState {
    private double[] location;            // ECEF position vector (x,y,z) (m)
    private double[] orientation;         // ECEF orientation vector (roll, pitch, yaw) (radians)
    private double[] linearVelocity;      // ECEF linear velocity vector (x,y,z) (m/s)
    private double[] linearAcceleration;  // ECEF linear acceleration vector (x,y,z) (m/s^2)
    private double[] angularVelocity;     // ECEF angular velocity vector (rollrate, pitchrate, yawrate) (radians/s)

    public EntityState() {
        location = new double[] {0d, 0d, 0d};
        orientation = new double[] {0d, 0d, 0d};
        linearVelocity = new double[] {0d, 0d, 0d};
        linearAcceleration = new double[] {0d, 0d, 0d};
        angularVelocity = new double[] {0d, 0d, 0d};
    }

    public EntityState(double[] location,
                       double[] orientation,
                       double[] linearVelocity,
                       double[] linearAcceleration,
                       double[] angularVelocity) {
        this.location = location;
        this.orientation = orientation;
        this.linearVelocity = linearVelocity;
        this.linearAcceleration = linearAcceleration;
        this.angularVelocity = angularVelocity;
    }

    /**
     * Creates an {@link EntityState} from common aeronautical parameters
     * <p>
     * Warning: Accelerometers or IMUs typically provide PROPER acceleration data as opposed to BODY acceleration
     * data. You must convert PROPER acceleration to BODY acceleration by subtracting free-fall acceleration
     * vector (0,0,-9.80665) from proper acceleration vector)
     *
     * @param latLonAlt position vector of entity in WGS84 (latitude in DD, longitude in DD, altitude in m above WGS84 ellipsoid)
     * @param orientationNed orientation/attitude vector of entity in NED frame (radians/s)
     * @param linearVelocityNed linear velocity vector of entity in NED frame (velocity north, velocity east, velocity down) (m/s)
     * @param linearAccelerationBody linear acceleration vector of entity in NED frame (x,y,z) (m/s^2)
     * @param angularVelocityBody angular velocity vector of entity in NED frame (rollrate, pitchrate, yawrate) (radians/s)
     * @return new {@link EntityState}
     */
    public static EntityState fromAeronauticalFrame(double[] latLonAlt,
                                                    double[] orientationNed,
                                                    double[] linearVelocityNed,
                                                    double[] linearAccelerationBody,
                                                    double[] angularVelocityBody) {

        final double lat = latLonAlt[0];
        final double lon = latLonAlt[1];
        final double alt = latLonAlt[2];
        final double roll = orientationNed[0];
        final double pitch = orientationNed[1];
        final double yaw = orientationNed[2];

        // convert latLonAlt to ECEF

        //% WGS84 ellipsoid constants:
//        a = 6378137;
//        e = 8.1819190842622e-2;
//
//        % intermediate calculation
//        % (prime vertical radius of curvature)
//        N = a ./ sqrt(1 - e^2 .* sin(lat).^2);
//
//        % results:
//        x = (N+alt) .* cos(lat) .* cos(lon);
//        y = (N+alt) .* cos(lat) .* sin(lon);
//        z = ((1-e^2) .* N + alt) .* sin(lat);
//        % Source: "Department of Defense World Geodetic System 1984"
//        %         Page 4-4
//        %         National Imagery and Mapping Agency
//        %         Last updated June, 2004
//        %         NIMA TR8350.2

        // assuming latLonAlt is WGS84

        // WGS84 ellipsoid constants
        final double a = 6378137;
        final double e = 8.1819190842622e-2;

        // prime vertical radius of curvature
        final double cosLat = Math.cos(lat);
        final double cosLon = Math.cos(lon);
        final double sinLat = Math.sin(lat);
        final double sinLon = Math.sin(lon);
        final double N = a / Math.sqrt(1.0 - e*e * sinLat*sinLat);

        // ECEF position
        final double[] position = new double[] {
                (N+alt) * cosLat * cosLon,
                (N+alt) * cosLat * sinLon,
                ((1.0-e*e) * N + alt) * sinLat
        };

        // BODY to NED rotation matrix
        final double cosR = Math.cos(roll);
        final double sinR = Math.sin(roll);
        final double cosP = Math.cos(pitch);
        final double sinP = Math.sin(pitch);
        final double cosY = Math.cos(yaw);
        final double sinY = Math.sin(yaw);
        final double[][] bodyToNedArray = {
                {cosY*cosP,   -sinY*cosR + cosY*sinP*sinR,  sinY*sinR + cosY*sinP*cosR},
                {sinY*cosP,   cosY*cosR + sinY*sinP*sinR,   -cosY*sinR + sinY*sinP*cosR},
                {-sinP,       cosP*sinR,                    cosP*cosR}
        };
        // for linear acceleration and angular velocity (which is in body coordinates)
        final RealMatrix bodyToNed = MatrixUtils.createRealMatrix(bodyToNedArray);


        // NED to ECEF rotation matrix
        final double[][] nedToEcefArray = {
                {-sinLat*cosLon,  -sinLat*sinLon,  cosLat},
                {-sinLon,         cosLon,          0.0},
                {-cosLat*cosLon,  -cosLat*sinLon,  -sinLat}
        };
        final RealMatrix nedToEcef = MatrixUtils.createRealMatrix(nedToEcefArray);

        // BODY to ECEF rotation matrix
        final RealMatrix bodyToEcef = nedToEcef.multiply(bodyToNed);



        // convert orientation in NED to ECEF
        final double[] orientation = nedToEcef.operate(orientationNed);

        // convert linear velocity in NED to ECEF
        final double[] linearVelocity = nedToEcef.operate(linearVelocityNed);

        // convert linear acceleration in BODY to ECEF
        final double[] linearAcelleration = bodyToEcef.operate(linearAccelerationBody);

        // convert angular velocity in BODY to ECEF
        final double[] angularVelocity = bodyToEcef.operate(angularVelocityBody);

        return new EntityState(position, orientation, linearVelocity, linearAcelleration, angularVelocity);
    }

    /**
     * Returns an instance of {@link EntityState} where all fields are in common aeronautical frames.
     *
     * @return
     */
    public EntityState toAeronauticalFrame() {
        return new EntityState();
    }

    public double[] getLocation() {
        return location;
    }

    public double[] getOrientation() {
        return orientation;
    }

    public double[] getLinearVelocity() {
        return linearVelocity;
    }

    public double[] getLinearAcceleration() {
        return linearAcceleration;
    }

    public double[] getAngularVelocity() {
        return angularVelocity;
    }
}
