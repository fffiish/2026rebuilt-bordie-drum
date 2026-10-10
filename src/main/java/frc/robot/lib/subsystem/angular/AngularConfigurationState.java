package frc.robot.lib.subsystem.angular;

import java.util.function.BooleanSupplier;

/** Disabled-only, acknowledged configuration with bounded retries. No controller dependencies. */
final class AngularConfigurationState {
  private boolean pending = true;
  private boolean ready;
  private String error = "Configuration not verified";
  private double lastAttempt = Double.NEGATIVE_INFINITY;
  private boolean referenceValid;
  private long generation;

  void request() {
    pending = true;
    ready = false;
    error = "Waiting for disabled configuration/readback";
    lastAttempt = Double.NEGATIVE_INFINITY;
  }

  void reject(String reason) {
    ready = false;
    pending = false;
    error = reason;
  }

  void service(boolean disabled, double now, BooleanSupplier applyAndVerify) {
    if (!disabled || !pending || now - lastAttempt < 1.0) return;
    lastAttempt = now;
    try {
      ready = applyAndVerify.getAsBoolean();
      pending = !ready;
      error = ready ? "" : "Configuration acknowledgement or readback failed";
    } catch (RuntimeException exception) {
      ready = false;
      pending = true;
      error = "Configuration failed: " + exception.getClass().getSimpleName();
    }
  }

  boolean ready() {
    return ready;
  }

  boolean pending() {
    return pending;
  }

  String error() {
    return error;
  }

  boolean referenceValid() {
    return referenceValid;
  }

  long generation() {
    return generation;
  }

  void confirmReference() {
    referenceValid = ready && !pending;
  }

  void invalidateController(String reason) {
    if (ready || referenceValid) generation++;
    referenceValid = false;
    ready = false;
    pending = true;
    error = reason;
    // Preserve retry deadline; a disconnected status every cycle must not create a CAN flood.
  }
}
