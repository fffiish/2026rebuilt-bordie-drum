package frc.robot.lib.subsystem;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Connection health from cached status reads, with no firmware requests in the robot loop. */
public final class SparkConnectionMonitor {
  private final String logKey;
  private final Supplier<REVLibError> lastError;
  private int readCount;
  private boolean invalidData;
  private REVLibError cycleError = REVLibError.kOk;

  /** Create after initial configuration so startup parameter writes keep their normal retries. */
  public SparkConnectionMonitor(SparkBase spark) {
    this(spark.getDeviceId(), spark::getLastError);
    // Apply only after startup configuration. Periodic telemetry uses cached CAN status frames;
    // runtime parameter requests must never wait on a missing device on the scheduler thread.
    spark.setCANTimeout(0);
    spark.setCANMaxRetries(0);
    // REV enforces a minimum age limit of 2.1 times each frame's configured period. Zero selects
    // that limit rather than allowing the default 500 ms of stale data to look connected.
    spark.setPeriodicFrameTimeout(0);
  }

  SparkConnectionMonitor(int deviceId, Supplier<REVLibError> lastError) {
    logKey = "Health/SparkCAN" + deviceId;
    this.lastError = lastError;
  }

  /** Start before telemetry reads. This method performs no CAN operation. */
  public void beginCycle() {
    cycleError = REVLibError.kOk;
    readCount = 0;
    invalidData = false;
  }

  /** REVLib errors are per thread: inspect immediately after each read, before another device. */
  public void checkLastError(double value) {
    REVLibError error = lastError.get();
    readCount++;
    invalidData |= !Double.isFinite(value);
    if (error != REVLibError.kOk) {
      cycleError = error;
    }
  }

  public boolean isConnected() {
    Logger.recordOutput(logKey + "/LastReadError", cycleError.toString());
    Logger.recordOutput(logKey + "/StatusReadCount", readCount);
    Logger.recordOutput(logKey + "/InvalidData", invalidData);
    return hasHealthyReadings();
  }

  boolean hasHealthyReadings() {
    return readCount > 0 && !invalidData && cycleError == REVLibError.kOk;
  }
}
