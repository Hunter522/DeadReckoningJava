package org.hmorgan.deadreckoning;

import java.time.Duration;
import java.time.Instant;

/**
 * Implementation of the dead reckoning algorithm RVW as defined in IEEE 1278.1-1995 (DIS).
 *
 * This is the RVW algorithm which uses rotational body (R), change in velocity (V), and world-referenced (W).
 * This implementation uses a smoothing algorithm using BLA BLA curves to smooth updates in kinematic state. Another
 * useful feature is that it uses a decay over a set period of time since last kinematic state update. This is useful
 * for when it is necessary to visually detect when an entity is no longer receiving state updates.
 *
 * @author Hunter N. Morgan
 */
public class DeadReckoningRVW implements DeadReckoningAlgorithm {
    private EntityState oldEntityState;
    private EntityState currentEntityState;

    private Instant timeLastUpdated;
    private double timeDelta;
    private boolean useAccelerationDecay;

    private final double ACCELERATION_DECAY_INTERVAL = 5.0;
    private final double INTERPOLATION_INTERVAL = 1.0; // seconds to interpolate from old state to new state

    public DeadReckoningRVW() {
        useAccelerationDecay = true;
    }

    public DeadReckoningRVW(boolean useAccelerationDecay) {
        this.useAccelerationDecay = useAccelerationDecay;
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

            // acceleration decay (decelleration?)
            // this just artificially slows down time down to a halt over a period of time
            // since the last state update
            if(useAccelerationDecay) {
                double interpFrac = (timeDelta / ACCELERATION_DECAY_INTERVAL);
                if(interpFrac > 1.0)
                    interpFrac = 1.0;
                timeDelta = easeOutSine(timeLastUpdated.toEpochMilli(),
                                        timeLastUpdated.toEpochMilli() + ACCELERATION_DECAY_INTERVAL*1000,
                                        interpFrac);
            }

            // position dead reckoning
            // x(t) = x_0 + v_0*t + 0.5*a*t^2
            final double[] drPosition = new double[] {
                    entityState.getLocation()[0] + entityState.getLinearVelocity()[0]*timeDelta + 0.5*entityState.getLinearAcceleration()[0]*timeDelta*timeDelta,
                    entityState.getLocation()[1] + entityState.getLinearVelocity()[1]*timeDelta + 0.5*entityState.getLinearAcceleration()[1]*timeDelta*timeDelta,
                    entityState.getLocation()[2] + entityState.getLinearVelocity()[2]*timeDelta + 0.5*entityState.getLinearAcceleration()[2]*timeDelta*timeDelta
            };

            // orientation dead reckoning is simplified (not using the more complicated one in the DIS standard)
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

    private static double easeOutSine(double a, double b, double f) {
        return (b-a) * Math.sin(f * (Math.PI/2.0)) + a;
    }

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
