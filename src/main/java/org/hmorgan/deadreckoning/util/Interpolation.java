package org.hmorgan.deadreckoning.util;


/**
 * Collection of static interpolation functions
 *
 * @see http://paulbourke.net/miscellaneous/interpolation/
 */
public class Interpolation {

    /**
     * Linear interpolate between y1 and y2 using mu
     *
     * mu must be in domain [0,1], if outside then this will compute the extrapolation
     *
     * @param y1
     * @param y2
     * @param mu
     * @return
     */
    public static double linearInterpolate(double y1, double y2, double mu) {
        return(y1*(1-mu)+y2*mu);
    }

    /**
     * Cubic interpolate between y1 and y2 with points on either side of the endpoints y0 and y3 using mu
     *
     * y0 and y3 can be made up values using the slope at each endpoint y1 and y2
     *
     * mu must be in domain [0,1], if outside then this will compute the extrapolation
     *
     * @param y0
     * @param y1
     * @param y2
     * @param y3
     * @param mu
     * @return
     */
    public static double cubicInterpolate(double y0, double y1, double y2, double y3, double mu) {
        double a0,a1,a2,a3,mu2;

        mu2 = mu*mu;
        a0 = y3 - y2 - y0 + y1;
        a1 = y0 - y1 - a0;
        a2 = y2 - y0;
        a3 = y1;

        return(a0*mu*mu2+a1*mu2+a2*mu+a3);
    }


    /**
     * Catmull-Rom spline interpolate between y1 and y2 with points on either side of the endpoints y0 and y3 using mu
     *
     * y0 and y3 can be made up values using the slope at each endpoint y1 and y2
     *
     * mu must be in domain [0,1], if outside then this will compute the extrapolation
     *
     * @param y0
     * @param y1
     * @param y2
     * @param y3
     * @param mu
     * @return
     */
    public static double catmullRomSplineInterpolate(double y0, double y1, double y2, double y3, double mu) {
        double a0,a1,a2,a3,mu2;

        mu2 = mu*mu;
        a0 = -0.5*y0 + 1.5*y1 - 1.5*y2 + 0.5*y3;
        a1 = y0 - 2.5*y1 + 2*y2 - 0.5*y3;
        a2 = -0.5*y0 + 0.5*y2;
        a3 = y1;

        return(a0*mu*mu2+a1*mu2+a2*mu+a3);
    }

    /**
     * Hermite interpolate between y1 and y2 with points on either side y0 and y3
     *
     * Tension: 1 is high, 0 normal, -1 is low
     * Bias: 0 is even,
     *       positive is towards first segment,
     *       negative towards the other
     *
     * @param y0
     * @param y1
     * @param y2
     * @param y3
     * @param mu
     * @param tension
     * @param bias
     * @return
     */
    public static double hermiteInterpolate(double y0,double y1, double y2,double y3, double mu, double tension, double bias) {
        double m0,m1,mu2,mu3;
        double a0,a1,a2,a3;

        mu2 = mu * mu;
        mu3 = mu2 * mu;
        m0  = (y1-y0)*(1+bias)*(1-tension)/2;
        m0 += (y2-y1)*(1-bias)*(1-tension)/2;
        m1  = (y2-y1)*(1+bias)*(1-tension)/2;
        m1 += (y3-y2)*(1-bias)*(1-tension)/2;
        a0 =  2*mu3 - 3*mu2 + 1;
        a1 =    mu3 - 2*mu2 + mu;
        a2 =    mu3 -   mu2;
        a3 = -2*mu3 + 3*mu2;

        return(a0*y1+a1*m0+a2*m1+a3*y2);
    }
}
