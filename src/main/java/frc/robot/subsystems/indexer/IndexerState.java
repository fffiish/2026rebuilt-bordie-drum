package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.indexer.IndexerConstants;

/** Every speed the indexer wheel row can run at. */
public enum IndexerState {
  /** Stopped. */
  kIdle(RotationsPerSecond.of(0.0)),

  /** Slow forward — pull FUEL out of the hopper and stage it at the throat. */
  kIntaking(IndexerConstants.kIntaking),

  /** Full forward — push staged FUEL into the flywheel. */
  kFeeding(IndexerConstants.kFeeding),

  /**
   * Reverse. The hopper is passive, so this wheel row is the only thing that can break up a jam —
   * bind it to a button rather than relying on automatic recovery.
   */
  kUnjamming(IndexerConstants.kUnjamming);

  private final AngularVelocity velocity;

  IndexerState(AngularVelocity velocity) {
    this.velocity = velocity;
  }

  public AngularVelocity getVelocity() {
    return velocity;
  }
}
