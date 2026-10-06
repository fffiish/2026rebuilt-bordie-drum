package frc.robot.subsystems.shooter;

/**
 * The hood has exactly two mechanical positions, set by a pair of 12 V linear actuators (one per
 * side) that pull the back of the hood forward or release it.
 *
 * <p>There is no continuum here, so there is no distance-to-angle interpolation. Range within a
 * hood position is controlled entirely by flywheel RPM.
 */
public enum HoodState {
  /** Actuators retracted — the hood's resting angle. */
  kNear(false),

  /** Actuators extended — back of the hood pulled forward. */
  kFar(true);

  private final boolean extended;

  HoodState(boolean extended) {
    this.extended = extended;
  }

  public boolean isExtended() {
    return extended;
  }
}
