package frc.robot.lib.subsystem;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.REVLibError;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class SparkConnectionMonitorTest {
  @Test
  void emptyCyclesDoNotReportAConnectionOrRequestDeviceData() {
    var calls = new AtomicInteger();
    var monitor =
        new SparkConnectionMonitor(
            3,
            () -> {
              calls.incrementAndGet();
              return REVLibError.kOk;
            });
    for (int i = 0; i < 10000; i++) {
      monitor.beginCycle();
      assertFalse(monitor.hasHealthyReadings());
    }
    assertEquals(0, calls.get());
  }

  @Test
  void disconnectStaysVisibleDespiteLaterHealthyReadsAndRecoversNextCycle() {
    var error = new AtomicReference<>(REVLibError.kOk);
    var monitor = new SparkConnectionMonitor(37, error::get);
    monitor.beginCycle();
    monitor.checkLastError(0.0);
    assertTrue(monitor.hasHealthyReadings());
    error.set(REVLibError.kTimeout);
    monitor.checkLastError(0.0);
    error.set(REVLibError.kOk);
    monitor.checkLastError(12.0);
    assertFalse(monitor.hasHealthyReadings());
    monitor.beginCycle();
    assertFalse(monitor.hasHealthyReadings());
    monitor.checkLastError(12.0);
    assertTrue(monitor.hasHealthyReadings());
  }

  @Test
  void invalidValuesCannotReportAHealthyDevice() {
    var monitor = new SparkConnectionMonitor(3, () -> REVLibError.kOk);
    for (double value :
        new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      monitor.beginCycle();
      monitor.checkLastError(value);
      monitor.checkLastError(0.0);
      assertFalse(monitor.hasHealthyReadings());
    }
    monitor.beginCycle();
    monitor.checkLastError(0.0);
    assertTrue(monitor.hasHealthyReadings());
  }

  @Test
  void perThreadErrorsAreCapturedBeforeAnotherDeviceReadClearsThem() {
    var error = new AtomicReference<>(REVLibError.kOk);
    var offline = new SparkConnectionMonitor(37, error::get);
    var online = new SparkConnectionMonitor(36, error::get);
    offline.beginCycle();
    online.beginCycle();
    error.set(REVLibError.kTimeout);
    offline.checkLastError(0.0);
    error.set(REVLibError.kOk);
    online.checkLastError(0.0);
    assertFalse(offline.hasHealthyReadings());
    assertTrue(online.hasHealthyReadings());
  }
}
