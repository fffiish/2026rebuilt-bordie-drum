package frc.robot.lib.subsystem.angular;

import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class AngularConfigurationStateTest {
  @Test
  void enabledNeverAppliesOrAcknowledgesPendingConfiguration() {
    var state = new AngularConfigurationState();
    var calls = new AtomicInteger();
    state.service(
        false,
        1.0,
        () -> {
          calls.incrementAndGet();
          return true;
        });
    assertEquals(0, calls.get());
    assertFalse(state.ready());
    assertTrue(state.pending());
    state.service(
        true,
        2.0,
        () -> {
          calls.incrementAndGet();
          return true;
        });
    assertEquals(1, calls.get());
    assertTrue(state.ready());
    assertFalse(state.pending());
    state.request();
    state.service(
        false,
        3.0,
        () -> {
          calls.incrementAndGet();
          return true;
        });
    assertFalse(state.ready());
    assertEquals(1, calls.get());
  }

  @Test
  void failedAcknowledgementRetriesOnlyDisabledAndAtBoundedRate() {
    var state = new AngularConfigurationState();
    var calls = new AtomicInteger();
    state.service(
        true,
        1,
        () -> {
          calls.incrementAndGet();
          return false;
        });
    state.service(
        true,
        1.1,
        () -> {
          calls.incrementAndGet();
          return true;
        });
    assertEquals(1, calls.get());
    assertFalse(state.ready());
    assertTrue(state.pending());
    assertFalse(state.error().isEmpty());
    state.service(
        false,
        3,
        () -> {
          calls.incrementAndGet();
          return true;
        });
    assertEquals(1, calls.get());
    state.service(
        true,
        4,
        () -> {
          calls.incrementAndGet();
          return true;
        });
    assertEquals(2, calls.get());
    assertTrue(state.ready());
    assertEquals("", state.error());
  }

  @Test
  void exceptionOrInvalidRequestCannotLeaveOldConfigurationMarkedReady() {
    var state = new AngularConfigurationState();
    state.service(
        true,
        1,
        () -> {
          throw new IllegalStateException("Disconnected");
        });
    assertFalse(state.ready());
    assertTrue(state.pending());
    state.service(true, 2, () -> true);
    assertTrue(state.ready());
    state.reject("Nonfinite gain request");
    assertFalse(state.ready());
    assertFalse(state.pending());
    state.service(true, 3, () -> true);
    assertFalse(state.ready());
    state.request();
    state.service(true, 4, () -> true);
    assertTrue(state.ready());
  }

  @Test
  void controllerFaultLosesReferenceAndReconfigurationNeverReseedsIt() {
    var state = new AngularConfigurationState();
    assertFalse(state.referenceValid());
    state.confirmReference();
    assertFalse(state.referenceValid(), "Unverified configuration cannot qualify a reference");
    state.service(true, 1, () -> true);
    state.confirmReference();
    assertTrue(state.referenceValid());
    state.invalidateController("Controller reset");
    assertFalse(state.ready());
    assertFalse(state.referenceValid());
    assertEquals(1, state.generation());
    state.service(false, 2, () -> true);
    assertFalse(state.ready());
    state.service(true, 3, () -> true);
    assertTrue(state.ready());
    assertFalse(state.referenceValid(), "Disabled reapply must not bless the old physical zero");
    state.confirmReference();
    assertTrue(state.referenceValid());
  }
}
