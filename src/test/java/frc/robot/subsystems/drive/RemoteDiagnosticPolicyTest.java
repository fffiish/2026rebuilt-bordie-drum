package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class RemoteDiagnosticPolicyTest {
  @Test
  void heartbeatAcceptsFreshInclusiveBoundaryAndRejectsStaleOrFuture() {
    assertTrue(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 1.0));
    assertTrue(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 0.75));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 0.749));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 1.01));
    assertTrue(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 0.5, 0.5));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 0.49, 0.5));
  }

  @Test
  void heartbeatRejectsNegativeNonfiniteAndInvalidMaxAge() {
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(-1.0, -1.0));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(0.0, -0.01));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(Double.NaN, 0.0));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(1.0, Double.NaN));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(Double.POSITIVE_INFINITY, 1.0));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(1.0, Double.NEGATIVE_INFINITY));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 1.0, 0.0));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 1.0, -0.25));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 1.0, Double.NaN));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(1.0, 1.0, Double.POSITIVE_INFINITY));
  }

  @Test
  void disabledOrFaultedRequestsAreConsumedBeforeRecovery() {
    var gate = new RemoteDiagnosticPolicy.RequestGate();
    assertFalse(gate.consume(1, false));
    assertFalse(gate.consume(1, true));
    assertEquals(1, gate.lastConsumedNonce());
    assertTrue(gate.consume(2, true));
    assertFalse(gate.consume(3, false));
    assertFalse(gate.consume(3, true));
    assertTrue(gate.consume(4, true));
  }

  @Test
  void heldRequestDoesNotRepeatAndOlderRequestsNeverStart() {
    var gate = new RemoteDiagnosticPolicy.RequestGate();
    assertTrue(gate.consume(10, true));
    for (int i = 0; i < 100; i++) {
      assertFalse(gate.consume(10, true));
    }
    assertFalse(gate.consume(9, true));
    assertFalse(gate.consume(1, true));
    assertTrue(gate.consume(11, true));
    assertFalse(gate.consume(10, true));
  }

  @Test
  void invalidNoncesCannotActivateOrAdvanceTheGate() {
    var gate = new RemoteDiagnosticPolicy.RequestGate();
    for (double nonce : new double[] {
      0, -1, 0.5, 1.5, Double.NaN, Double.POSITIVE_INFINITY,
      Double.NEGATIVE_INFINITY, 9_007_199_254_740_992.0
    }) {
      assertFalse(gate.consume(nonce, true));
      assertEquals(0, gate.lastConsumedNonce());
    }
    assertTrue(gate.consume(1, true));
    assertTrue(gate.consume(9_007_199_254_740_991.0, true));
    assertFalse(gate.consume(9_007_199_254_740_991.0, true));
  }
}
