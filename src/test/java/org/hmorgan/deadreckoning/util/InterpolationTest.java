package org.hmorgan.deadreckoning.util;

import org.junit.jupiter.api.*;

import static org.junit.jupiter.api.Assertions.*;

class InterpolationTest {

    private static final double EPSILON = 0.000001;

    @BeforeEach
    void setUp() {
    }

    @AfterEach
    void tearDown() {
    }

    @Nested
    @DisplayName("test linearInterpolate")
    class TestLinearInterpolate {
        @Test
        @DisplayName("endpoints input should return endpoints")
        void testBounds() {
            final double y1 = 0.0;
            final double y2 = 10.0;

            // test bounds
            assertEquals(y1, Interpolation.linearInterpolate(y1, y2, 0.0), EPSILON);
            assertEquals(y2, Interpolation.linearInterpolate(y1, y2, 1.0), EPSILON);
        }

        @Test
        @DisplayName("middle should return middle of endpoints")
        void testMiddle() {
            final double y1 = 0.0;
            final double y2 = 10.0;

            // test middle
            assertEquals(5.0, Interpolation.linearInterpolate(y1, y2, 0.5), EPSILON);
        }

        @Test
        @DisplayName("should extrapolate if mu > 1")
        void testExtrapolate() {
            final double y1 = 0.0;
            final double y2 = 10.0;

            // test extrapolate
            assertEquals(20.0, Interpolation.linearInterpolate(y1, y2, 2.0), EPSILON);
        }
    }

    @Nested
    @DisplayName("test cubicInterpolate")
    class TestCubicInterpolate {

    }

    @Nested
    @DisplayName("test catmullRomSplineInterpolate")
    class testCatmullRomSplineInterpolate {
        @Test
        void testExample() {
            final double x1 = 0.0;
            final double y1 = 0.0;
            final double x2 = 10.0;
            final double y2 = 100.0;
            final double samples = 100;

            for(int i = 0; i <= samples; i++) {
                final double rx = Interpolation.catmullRomSplineInterpolate(-10.0, x1, x2, 20.0, i / samples);
                final double ry = Interpolation.catmullRomSplineInterpolate(0, y1, y2, 100.0, i / samples);
                System.out.println(rx + "," + ry);
            }
        }
    }

    @Nested
    @DisplayName("test hermiteInterpolate")
    class testHermiteInterpolate {

    }
}