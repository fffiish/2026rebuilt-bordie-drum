package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.indexer.IndexerConstants;

/** Every open-loop voltage the indexer wheel row can run at. */
public enum IndexerState {
  /** Stopped. */
  kIdle(Volts.of(0.0)),

  /** Slow forward — pull FUEL out of the hopper and stage it at the throat. */
  kIntaking(IndexerConstants.kIntakingVoltage),

  /** Full forward — push staged FUEL into the flywheel. */
  kFeeding(IndexerConstants.kFeedingVoltage),

  /**
   * Reverse. The hopper is passive, so this wheel row is the only thing that can break up a jam —
   * bind it to a button rather than relying on automatic recovery.
   */
  kUnjamming(IndexerConstants.kUnjammingVoltage);

  private final Voltage voltage;

  IndexerState(Voltage voltage) {
    this.voltage = voltage;
  }

  public Voltage getVoltage() {
    return voltage;
  }
}
