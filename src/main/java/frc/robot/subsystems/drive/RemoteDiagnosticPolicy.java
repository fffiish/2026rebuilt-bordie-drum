package frc.robot.subsystems.drive;

/** Pure guards for explicit, single-run remote diagnostic requests. */
public final class RemoteDiagnosticPolicy {
  public static final double MAX_HEARTBEAT_AGE_SEC = 0.25;
  private static final double MAX_SAFE_NONCE = 9_007_199_254_740_991.0;

  private RemoteDiagnosticPolicy() {}

  /** Heartbeat timestamps and now must use the same monotonic clock. */
  public static boolean heartbeatFresh(double nowSec, double lastHeartbeatSec) {
    return heartbeatFresh(nowSec, lastHeartbeatSec, MAX_HEARTBEAT_AGE_SEC);
  }

  public static boolean heartbeatFresh(double nowSec, double lastHeartbeatSec, double maxAgeSec) {
    double age = nowSec - lastHeartbeatSec;
    return Double.isFinite(nowSec)
        && nowSec >= 0.0
        && Double.isFinite(lastHeartbeatSec)
        && lastHeartbeatSec >= 0.0
        && Double.isFinite(maxAgeSec)
        && maxAgeSec > 0.0
        && age >= 0.0
        && age <= maxAgeSec;
  }

  /**
   * A new positive integer request is consumed even when ineligible, so enabling or clearing a
   * fault cannot start an old request. Nonces increase throughout the robot program's lifetime.
   */
  public static final class RequestGate {
    private double lastConsumedNonce;

    public boolean consume(double nonce, boolean eligible) {
      if (!Double.isFinite(nonce)
          || nonce <= 0.0
          || nonce > MAX_SAFE_NONCE
          || nonce != Math.rint(nonce)
          || nonce <= lastConsumedNonce) {
        return false;
      }
      lastConsumedNonce = nonce;
      return eligible;
    }

    public double lastConsumedNonce() {
      return lastConsumedNonce;
    }
  }
}
