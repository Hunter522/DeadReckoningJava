package org.hmorgan.deadreckoning;

import java.time.Duration;
import java.time.Instant;

/**
 * Implementation of a dead reckoning algorithm similar to {@link DeadReckoningRVW} but uses
 * non-uniform circular motion and linear motion equations combined.
 *
 * This produced a more realistic dead reckoned position for entities such as aircraft when
 * they are moving in circular motions (banking, turning, etc.).
 *
 * The algorithm employs either non-uniform circular motion or linear motion equations like a
 * piecewise function. When the angular velocity magnitude is >= 0.5 radians/sec, then non-uniform
 * circular motion is used, otherwise if angular velocity magnitude is < 0.5 radians/sec, then
 * linear motion is used.
 *
 * Smoothing is done between kinematic state updates using cubic splines over a 5s interval.
 *
 * This implementation is considered thread-safe.
 *
 * @author Hunter N. Morgan
 */
public class DeadReckoningRVWCM implements DeadReckoningAlgorithm {

    private EntityState oldEntityState;
    private EntityState currentEntityState;

    private Instant timeLastUpdated;
    private double timeDelta;

    private final double INTERPOLATION_INTERVAL = 1.0; // seconds to interpolate from old state to new state

    public DeadReckoningRVWCM() {

    }

    public void updateKinematicState(EntityState state) {
        synchronized (this) {
            if(oldEntityState == null) {
                currentEntityState = state;
                oldEntityState = currentEntityState;
            } else {
                oldEntityState = currentEntityState;
                currentEntityState = state;
            }

            timeDelta = 0.0;
            timeLastUpdated = Instant.now();
        }
    }

    public EntityState getCurrentDeadReckonedState() {
        synchronized (this) {
            if(oldEntityState == null || currentEntityState == null) {
                return new EntityState();
            }

            timeDelta = Duration.between(timeLastUpdated, Instant.now()).toMillis() / 1000.0;

            final EntityState entityState = interpolateState(oldEntityState, currentEntityState);

            // position dead reckoning
            // x(t) = x_0 + v_0*t + 0.5*a*t^2
            final double[] drPosition = new double[] {
                    entityState.getLocation()[0] + entityState.getLinearVelocity()[0]*timeDelta + 0.5*entityState.getLinearAcceleration()[0]*timeDelta*timeDelta,
                    entityState.getLocation()[1] + entityState.getLinearVelocity()[1]*timeDelta + 0.5*entityState.getLinearAcceleration()[1]*timeDelta*timeDelta,
                    entityState.getLocation()[2] + entityState.getLinearVelocity()[2]*timeDelta + 0.5*entityState.getLinearAcceleration()[2]*timeDelta*timeDelta
            };

            // orientation dead reckoning is simplified
            // theta(t) = theta_0 + omega_0*t
            final double[] drOrientation = new double[] {
                    entityState.getOrientation()[0] + entityState.getAngularVelocity()[0]*timeDelta,
                    entityState.getOrientation()[1] + entityState.getAngularVelocity()[1]*timeDelta,
                    entityState.getOrientation()[2] + entityState.getAngularVelocity()[2]*timeDelta
            };

            // not dead-reckoning linear velocity, linear acceleration, or angular velocity, so
            // the current state fields will be used in dead reckoned result
            return new EntityState(drPosition,
                                   drOrientation,
                                   currentEntityState.getLinearVelocity(),
                                   currentEntityState.getLinearAcceleration(),
                                   currentEntityState.getAngularVelocity());
        }
    }

    private EntityState interpolateState(EntityState a, EntityState b) {
        double interpFrac = (timeDelta / INTERPOLATION_INTERVAL);
        if(interpFrac > 1.0)
            interpFrac = 1.0;

        return new EntityState(linearInterpolate(a.getLocation(), b.getLocation(), interpFrac),
                               linearInterpolate(a.getOrientation(), b.getOrientation(), interpFrac),
                               linearInterpolate(a.getLinearVelocity(), b.getLinearVelocity(), interpFrac),
                               linearInterpolate(a.getLinearAcceleration(), b.getLinearAcceleration(), interpFrac),
                               linearInterpolate(a.getAngularVelocity(), b.getAngularVelocity(), interpFrac));
    }

    private static double linearInterpolate(double a, double b, double x) {
        return a + (b-a) * x;
    }

    private static double[] linearInterpolate(double[] a, double[] b, double x) {
        double[] array = new double[a.length];

        for(int i = 0; i < a.length; i++) {
            array[i] = linearInterpolate(a[i], b[i], x);
        }

        return array;
    }
//
//    private static double catmullRomSplineInterpolate() {
//        // since catmull-rom splines require 2 endpoints, and 2 outer control points, we need
//        // to figure out the control points. One way is to simply create a point that is along the slope
//        // of each endpoint. Since we're talking about positions here in 3D space, we can simply use the
//        // first derivative (linear velocity) to estimate the slope and generate these control points
//        // so v(t) = v_0 + 2at
//        // here we use a t = 0 and t = INTERPOLATION_INTERVAL
//        // then we calculate control point position using x(t) = x_0 + v_0*t + 0.5*a*t^2
//        //   where t = -1 and t = INTERPOLATION_INTERVAL+1
//    }
//
//    /**
//     *
//     * http://paulbourke.net/miscellaneous/interpolation/
//     *
//     * @param y0
//     * @param y1
//     * @param y2
//     * @param y3
//     * @param mu
//     * @return
//     */
//    private static double catmullRomSplineInterpolate(double y0, double y1, double y2, double y3, double mu) {
//        double a0,a1,a2,a3,mu2;
//
//        mu2 = mu*mu;
//        a0 = -0.5*y0 + 1.5*y1 - 1.5*y2 + 0.5*y3;
//        a1 = y0 - 2.5*y1 + 2*y2 - 0.5*y3;
//        a2 = -0.5*y0 + 0.5*y2;
//        a3 = y1;
//
//        return(a0*mu*mu2+a1*mu2+a2*mu+a3);
//    }
}
