package org.hmorgan.deadreckoning;

/**
 * Interface for all dead reckoning algorithms
 *
 * #author Hunter N. Morgan
 */
public interface DeadReckoningAlgorithm {

    /**
     * Updates this dead reckoning algorithm's kinematic state. It is up to the implementation
     * to smooth the transition to the new state.
     *
     * @param state the new kinematic state of the entity being dead reckoned
     */
    void updateKinematicState(EntityState state);

    /**
     * Calculates the current dead reckoned state and returns it
     *
     * @return the current dead reckoned state
     */
    EntityState getCurrentDeadReckonedState();
}
