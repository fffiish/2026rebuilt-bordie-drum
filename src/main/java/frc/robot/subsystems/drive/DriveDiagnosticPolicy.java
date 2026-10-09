package frc.robot.subsystems.drive;

import java.util.ArrayDeque;
import java.util.Arrays;

/** Hardware-independent guards for the operator-held drivetrain diagnostic. */
public final class DriveDiagnosticPolicy {
  public static final double MAX_SAMPLE_AGE_SEC = 0.1;
  public static final double MAX_STEER_PULSE_SEC = 0.150;
  public static final double MAX_STEER_PULSE_TRAVEL_RAD = Math.toRadians(2.0);
  private static final double ERROR_DEADBAND_RAD = Math.toRadians(1.0);

  private DriveDiagnosticPolicy() {}

  /** Signed shortest angular displacement, including both endpoints at +/- pi. */
  public static double wrapRadians(double angle) {
    return Math.IEEEremainder(angle, 2.0 * Math.PI);
  }

  public static boolean samplesFresh(double nowSec, double lastSampleSec) {
    double age = nowSec - lastSampleSec;
    return Double.isFinite(nowSec)
        && Double.isFinite(lastSampleSec)
        && age >= 0.0
        && age <= MAX_SAMPLE_AGE_SEC;
  }

  public static boolean sensorsHealthy(
      boolean driveConnected,
      boolean turnConnected,
      boolean encoderConnected,
      boolean turnSeeded) {
    return driveConnected && turnConnected && encoderConnected && turnSeeded;
  }

  public static boolean operatorEligible(boolean testEnabled, boolean deadman) {
    return testEnabled && deadman;
  }

  public static boolean steerPulseComplete(double elapsedSec, double travelRad) {
    return !Double.isFinite(elapsedSec)
        || !Double.isFinite(travelRad)
        || elapsedSec < 0.0
        || elapsedSec >= MAX_STEER_PULSE_SEC
        || Math.abs(travelRad) >= MAX_STEER_PULSE_TRAVEL_RAD;
  }

  /** Qualification uses the latest 250 loop periods and requires at least 100 samples. */
  public static final class TimingWindow {
    private static final int CAPACITY = 250;
    private static final int REQUIRED_SAMPLES = 100;
    private final double[] periods = new double[CAPACITY];
    private int count;
    private int next;
    private double lastSampleSec = Double.NaN;

    public void add(double periodMs, double nowSec) {
      if (!Double.isFinite(periodMs)
          || periodMs <= 0.0
          || !Double.isFinite(nowSec)
          || (Double.isFinite(lastSampleSec) && nowSec < lastSampleSec)) {
        clear();
        return;
      }
      periods[next] = periodMs;
      next = (next + 1) % CAPACITY;
      count = Math.min(count + 1, CAPACITY);
      lastSampleSec = nowSec;
    }

    public void clear() {
      count = 0;
      next = 0;
      lastSampleSec = Double.NaN;
    }

    public int sampleCount() {
      return count;
    }

    public double medianMs() {
      if (count == 0) {
        return Double.NaN;
      }
      double[] sorted = sortedPeriods();
      return count % 2 == 0
          ? (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0
          : sorted[count / 2];
    }

    /** Nearest-rank 95th percentile. */
    public double p95Ms() {
      if (count == 0) {
        return Double.NaN;
      }
      return sortedPeriods()[(int) Math.ceil(count * 0.95) - 1];
    }

    public boolean ready(double nowSec) {
      return count >= REQUIRED_SAMPLES
          && samplesFresh(nowSec, lastSampleSec)
          && medianMs() < 25.0
          && p95Ms() < 40.0;
    }

    private double[] sortedPeriods() {
      double[] sorted = Arrays.copyOf(periods, count);
      Arrays.sort(sorted);
      return sorted;
    }
  }

  /** Releasing while eligible arms a subsequent press; mode/fault loss clears that arm. */
  public static final class DeadmanGate {
    private boolean armed;
    private boolean active;

    public boolean update(boolean testEnabled, boolean held, boolean healthy) {
      if (!testEnabled || !healthy) {
        cancel();
      } else if (!held) {
        armed = true;
        active = false;
      } else if (armed) {
        armed = false;
        active = true;
      }
      return active;
    }

    public void cancel() {
      armed = false;
      active = false;
    }
  }

  /** Latched protection for fixed-target steering steps; reset for every new target. */
  public static final class OscillationGuard {
    private final ArrayDeque<Double> crossings = new ArrayDeque<>();
    private double growthLimitRad = Math.toRadians(7.0);
    private double lastTimeSec = Double.NaN;
    private int lastSign;
    private boolean tripped;

    public void reset(double nowSec, double initialStepRad) {
      crossings.clear();
      lastSign = 0;
      lastTimeSec = nowSec;
      growthLimitRad = Math.max(Math.abs(initialStepRad), Math.toRadians(5.0))
          + Math.toRadians(2.0);
      tripped = !Double.isFinite(nowSec) || !Double.isFinite(initialStepRad);
    }

    public boolean update(double nowSec, double errorRad) {
      if (tripped) {
        return true;
      }
      if (!Double.isFinite(nowSec)
          || !Double.isFinite(errorRad)
          || !Double.isFinite(lastTimeSec)
          || nowSec < lastTimeSec) {
        tripped = true;
        return true;
      }
      lastTimeSec = nowSec;
      double error = wrapRadians(errorRad);
      if (Math.abs(error) > growthLimitRad) {
        tripped = true;
        return true;
      }
      while (!crossings.isEmpty() && nowSec - crossings.peekFirst() > 1.0) {
        crossings.removeFirst();
      }
      int sign = Math.abs(error) > ERROR_DEADBAND_RAD ? (error > 0.0 ? 1 : -1) : 0;
      if (sign != 0) {
        if (lastSign != 0 && sign != lastSign) {
          crossings.addLast(nowSec);
        }
        lastSign = sign;
      }
      tripped = crossings.size() > 3;
      return tripped;
    }
  }
}
