package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.subsystem.sensor.canrange.CANRangeSubsystem;
import frc.robot.lib.subsystem.sensor.currentsensor.CurrentSensorSubsystem;

/**
 * The wheel row that pulls FUEL out of the passive hopper and stages it at the shooter throat.
 *
 * <p>There is deliberately <strong>no ball counter</strong>. The 2026 game manual places no limit
 * on how many SCORING ELEMENTS a robot may control after the start of a match, so counting buys
 * nothing legally and a counter that drifts actively hurts — it refuses to intake when you are not
 * actually full. Two booleans answer every question worth asking: {@link #staged()} and {@link
 * #jammed()}.
 */
public class Indexer extends VirtualSubsystem {
  public void stopImmediately() {
    rollers.stopImmediately();
  }

  private static final IndexerState kDefaultState = IndexerState.kIdle;

  private final AngularSubsystem rollers;
  private final CANRangeSubsystem stagedSensor;
  private final CurrentSensorSubsystem jamSensor;

  private IndexerState targetState = kDefaultState;

  public Indexer(
      AngularSubsystem rollers, CANRangeSubsystem stagedSensor, CurrentSensorSubsystem jamSensor) {
    this.rollers = rollers;
    this.stagedSensor = stagedSensor;
    this.jamSensor = jamSensor;

    rollers.setDefaultCommand(rollers.openLoop(() -> targetState.getVoltage()));
  }

  /** Holds {@code state} while scheduled, then falls back to idle. Bind with {@code whileTrue}. */
  public Command set(IndexerState state) {
    return Commands.startEnd(
        () -> targetState = state,
        () -> {
          targetState = kDefaultState;
          rollers.stopImmediately();
        },
        this);
  }

  /** Latches {@code state} and finishes immediately. For auto sequences. */
  public Command setPersistent(IndexerState state) {
    return Commands.runOnce(() -> targetState = state);
  }

  public IndexerState getTargetState() {
    return targetState;
  }

  /** A ball is sitting at the throat, ready to be fed into the flywheel. */
  public Trigger staged() {
    return stagedSensor.withinThreshold();
  }

  /**
   * The wheel row is stalling. Derived from already-logged supply current, so it costs no hardware
   * and replays correctly.
   */
  public Trigger jammed() {
    return jamSensor.exceedsThreshold();
  }
}
