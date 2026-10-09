package frc.robot.lib.subsystem;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Bounded firmware probes plus immediate status-read error checks for REVLib 2026. */
public final class SparkConnectionMonitor {
  private final SparkBase spark;
  private final String logKey;
  private int firmwareVersion;
  private boolean firmwareHealthy;
  private double nextProbe;
  private double lastProbe;
  private double probeDurationMs;
  private REVLibError cycleError = REVLibError.kOk;

  /** Create after initial configuration so startup parameter writes keep their normal retries. */
  public SparkConnectionMonitor(SparkBase spark) {
    this.spark = spark;
    logKey = "Health/SparkCAN" + spark.getDeviceId();
    // Firmware queries request a CAN response. A missing mechanism must not delay every robot loop.
    // Runtime parameter requests get one bounded attempt; configuration failures remain visible.
    spark.setCANTimeout(5);
    spark.setCANMaxRetries(0);
    beginCycle();
  }

  /** Start before telemetry reads; firmware is requested no more than once per second. */
  public void beginCycle() {
    cycleError = REVLibError.kOk;
    double now = Timer.getFPGATimestamp();
    if (now >= nextProbe) {
      firmwareVersion = spark.getFirmwareVersion();
      REVLibError error = spark.getLastError();
      firmwareHealthy = firmwareVersion != 0 && error == REVLibError.kOk;
      cycleError = error;
      lastProbe = now;
      nextProbe = now + 1.0;
      probeDurationMs = (Timer.getFPGATimestamp() - now) * 1000.0;
    }
  }

  /** REVLib errors are per thread: inspect immediately after each read, before another device. */
  public void checkLastError() {
    REVLibError error = spark.getLastError();
    if (error != REVLibError.kOk) {
      cycleError = error;
    }
  }

  public boolean isConnected() {
    Logger.recordOutput(logKey + "/FirmwareVersion", firmwareVersion);
    Logger.recordOutput(logKey + "/FirmwareProbeHealthy", firmwareHealthy);
    Logger.recordOutput(logKey + "/LastReadError", cycleError.toString());
    Logger.recordOutput(logKey + "/FirmwareProbeAgeSec", Timer.getFPGATimestamp() - lastProbe);
    Logger.recordOutput(logKey + "/FirmwareProbeDurationMS", probeDurationMs);
    return firmwareHealthy && cycleError == REVLibError.kOk;
  }
}
