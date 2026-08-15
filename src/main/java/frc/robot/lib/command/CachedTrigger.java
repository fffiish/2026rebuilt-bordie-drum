package frc.robot.lib.command;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

/**
 * Trigger that caches a boolean supplier once per scheduler cycle.
 *
 * <p>Call {@link #refreshAll()} before {@code CommandScheduler.run()} each robot loop.
 */
public class CachedTrigger extends Trigger {
  private static final List<CachedTrigger> instances = new ArrayList<>();

  private final BooleanSupplier source;
  private final AtomicBoolean cachedValue;

  public CachedTrigger(BooleanSupplier source) {
    this(new AtomicBoolean(false), source);
  }

  private CachedTrigger(AtomicBoolean cachedValue, BooleanSupplier source) {
    super(cachedValue::get);
    this.source = source;
    this.cachedValue = cachedValue;
    this.cachedValue.set(false); // Causes error since classes not initialized
    instances.add(this);
  }

  public void refresh() {
    cachedValue.set(source.getAsBoolean());
  }

  public static void refreshAll() {
    for (CachedTrigger trigger : instances) {
      trigger.refresh();
    }
  }
}
