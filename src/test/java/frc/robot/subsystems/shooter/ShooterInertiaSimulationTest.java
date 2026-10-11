package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.lib.subsystem.angular.AngularIO.AngularIOInputs;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Characterizes acceptance failures as data; test success means the model/harness ran correctly.
 */
class ShooterInertiaSimulationTest {
  private record Filter(String name, int periodMs, int depth) {}

  private record PendingVoltage(int dueMs, double volts) {}

  private static final class PhysicalIO extends ShooterDriverBindingTest.RecordingIO {
    final DCMotor motor = DCMotor.getNeoVortex(1);
    final double inertia = ShooterConstants.kFlywheelMOI.in(KilogramSquareMeters);
    final double currentLimit =
        ShooterConstants.kFlywheelSparkFlexConfig.getSmartCurrentLimit().in(Amps);
    final double gearing =
        ShooterConstants.kFlywheelSparkFlexConfig.getMotorRotationsPerOutputRotations();
    final int motors = ShooterConstants.kFlywheelSparkFlexConfig.getFollowerIds().size() + 1;
    final double bus;
    final Filter filter;
    final int canPhaseMs;
    final int commandDelayMs;
    final double drag;
    final ArrayDeque<PendingVoltage> commands = new ArrayDeque<>();
    final double[] positionHistory;
    final double[] velocityHistory;
    int ms;
    double omega;
    double position;
    double activeVolts;
    double motorCurrent;
    double filtered;
    double reported;
    double velocitySum;
    double peakCurrent;

    PhysicalIO(
        double bus, Filter filter, int canPhaseMs, int commandDelayMs, double dragMultiplier) {
      this.bus = bus;
      this.filter = filter;
      this.canPhaseMs = canPhaseMs;
      this.commandDelayMs = commandDelayMs;
      // Calibrate viscous drag to the recorded 242 rad/s at 4.46 V (no balls).
      double calibrationCurrent = (4.46 - 242 * gearing / motor.KvRadPerSecPerVolt) / motor.rOhms;
      drag = dragMultiplier * motors * motor.KtNMPerAmp * calibrationCurrent * gearing / 242;
      assertTrue(drag > 0);
      positionHistory = new double[filter.periodMs + 1];
      velocityHistory = new double[filter.depth];
    }

    @Override
    public void setOpenLoop(edu.wpi.first.units.measure.Voltage voltage) {
      super.setOpenLoop(voltage);
      commands.addLast(new PendingVoltage(ms + commandDelayMs, requestedVolts));
    }

    @Override
    public void stop() {
      super.stop();
      commands.clear();
      commands.addLast(new PendingVoltage(ms + commandDelayMs, 0));
    }

    private void stepMillisecond() {
      while (!commands.isEmpty() && commands.peekFirst().dueMs <= ms) {
        activeVolts = Math.min(bus, Math.max(0, commands.removeFirst().volts));
      }
      // Zero request opens the motor circuit (coast). Smart phase-current limiting is idealized.
      motorCurrent =
          activeVolts == 0
              ? 0
              : Math.max(
                  0,
                  Math.min(
                      currentLimit,
                      (activeVolts - omega * gearing / motor.KvRadPerSecPerVolt) / motor.rOhms));
      peakCurrent = Math.max(peakCurrent, motorCurrent);
      double torque = motors * motor.KtNMPerAmp * motorCurrent * gearing;
      omega = Math.max(0, omega + (torque - drag * omega) / inertia * 0.001);
      position += omega * 0.001;
      ms++;
      // High-resolution Vortex/Flex encoder; supported firmware is assumed.
      double quantizedPosition =
          Math.rint(position * gearing / (2 * Math.PI) * 7168) / 7168 * (2 * Math.PI) / gearing;
      int oldSlot = Math.floorMod(ms - filter.periodMs, positionHistory.length);
      double velocity =
          ms < filter.periodMs
              ? 0
              : (quantizedPosition - positionHistory[oldSlot]) / (filter.periodMs * 0.001);
      positionHistory[ms % positionHistory.length] = quantizedPosition;
      int slot = ms % filter.depth;
      velocitySum += velocity - velocityHistory[slot];
      velocityHistory[slot] = velocity;
      filtered = velocitySum / filter.depth;
      if ((ms - canPhaseMs) % 20 == 0) reported = filtered;
    }

    @Override
    public void updateInputs(AngularIOInputs inputs) {
      for (int i = 0; i < 20; i++) stepMillisecond();
      measuredRps = reported / (2 * Math.PI);
      motorMeasuredRps = new double[] {measuredRps, measuredRps, -measuredRps, -measuredRps};
      super.updateInputs(inputs);
      inputs.busVolts = Volts.of(bus);
      inputs.appliedVolts = Volts.of(activeVolts);
      inputs.statorCurrent = Amps.of(motors * motorCurrent);
      assertTrue(Double.isFinite(omega) && omega >= 0);
      assertTrue(motorCurrent >= 0 && motorCurrent <= currentLimit);
      assertTrue(activeVolts >= 0 && activeVolts <= bus);
    }
  }

  @Test
  void physicalModelMatchesTorqueInertiaCoastAndMeasuredHoldingPoint() {
    var io = new PhysicalIO(12, new Filter("physics-check", 5, 1), 0, 0, 1);
    io.setOpenLoop(Volts.of(12));
    io.stepMillisecond();
    double expectedAcceleration = io.motors * io.motor.KtNMPerAmp * 37 / io.inertia;
    assertEquals(expectedAcceleration * 0.001, io.omega, 1e-12);
    double beforeCoast = io.omega;
    io.setOpenLoop(Volts.of(0));
    io.stepMillisecond();
    assertEquals(0, io.motorCurrent);
    assertEquals(beforeCoast * (1 - io.drag / io.inertia * 0.001), io.omega, 1e-12);
    io.omega = 242;
    io.setOpenLoop(Volts.of(4.46));
    for (int i = 0; i < 1000; i++) io.stepMillisecond();
    assertEquals(242, io.omega, 1e-10);
  }

  @Test
  void characterizeCorrectedInertiaAndFeedbackDelay() throws IOException {
    assertEquals(0.0019021577472898, ShooterConstants.kFlywheelMOI.in(KilogramSquareMeters), 1e-15);
    var filters =
        List.of(
            new Filter("pinned-default-assumption", 100, 64),
            new Filter("explicit-fast-assumption", 10, 2),
            new Filter("explicit-faster-assumption", 5, 1));
    Path out = Path.of("logs/generated/shooter-inertia");
    Files.createDirectories(out);
    StringBuilder summary =
        new StringBuilder(
            "filter,period_ms,depth,state,bus_volts,can_phase_ms,command_delay_ms,drag_multiplier,ready_seconds,min_rpm,max_rpm,target_rpm,recovery_seconds,peak_phase_amps,startup_pass,steady_pass,recovery_pass,stop_pass\n");
    for (var filter : filters) {
      for (var state : List.of(ShooterState.kShootingNear, ShooterState.kShootingSoft)) {
        for (double bus : new double[] {12, 8}) {
          for (int canPhase : new int[] {0, 10}) {
            for (int commandDelay : new int[] {0, 20}) {
              for (double dragMultiplier : new double[] {0.5, 1, 1.5}) {
                var io = new PhysicalIO(bus, filter, canPhase, commandDelay, dragMultiplier);
                try (var f = new ShooterDriverBindingTest.Fixture(false, io)) {
                  double target = state.getFlywheelVelocity().in(RadiansPerSecond);
                  int startMs = io.ms;
                  boolean traceEnabled = canPhase == 0 && commandDelay == 0 && dragMultiplier == 1;
                  StringBuilder trace =
                      new StringBuilder(
                          "seconds,true_rpm,reported_rpm,target_rpm,requested_volts,applied_volts,phase_amps\n");
                  f.shooter.setPersistent(state).schedule();
                  double readySeconds = Double.NaN;
                  double min = Double.POSITIVE_INFINITY;
                  double max = 0;
                  for (int tick = 1; tick <= 350; tick++) {
                    f.advanceTicks(1);
                    if (Double.isNaN(readySeconds) && f.shooter.atSpeed().getAsBoolean())
                      readySeconds = tick * 0.02;
                    if (tick > 100) {
                      min = Math.min(min, io.omega);
                      max = Math.max(max, io.omega);
                    }
                    if (traceEnabled) appendTrace(trace, io, startMs, target);
                  }
                  // Synthetic speed-loss impulse, not a modeled ball collision.
                  io.omega = Math.min(io.omega, target) * 0.8;
                  double recoverySeconds = Double.NaN;
                  for (int tick = 1; tick <= 50; tick++) {
                    f.advanceTicks(1);
                    if (Double.isNaN(recoverySeconds) && io.omega >= target * 0.9)
                      recoverySeconds = tick * 0.02;
                    if (traceEnabled) appendTrace(trace, io, startMs, target);
                  }
                  f.shooter.stopImmediately();
                  f.advanceTicks(1);
                  boolean stopPass = io.requestedVolts == 0;
                  assertTrue(stopPass, "Immediate stop must clear the voltage request");
                  f.advanceTicks(2);
                  assertEquals(0, io.activeVolts);
                  assertEquals(0, io.motorCurrent);
                  assertEquals(0, io.velocityRequests);
                  boolean startupPass = Double.isFinite(readySeconds) && readySeconds <= 2;
                  boolean steadyPass = min >= target * 0.9 && max <= target * 1.1;
                  boolean recoveryPass = Double.isFinite(recoverySeconds) && recoverySeconds <= 1;
                  summary.append(
                      String.format(
                          java.util.Locale.ROOT,
                          "%s,%d,%d,%s,%.1f,%d,%d,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%b,%b,%b,%b%n",
                          filter.name,
                          filter.periodMs,
                          filter.depth,
                          state,
                          bus,
                          canPhase,
                          commandDelay,
                          dragMultiplier,
                          readySeconds,
                          min * 60 / (2 * Math.PI),
                          max * 60 / (2 * Math.PI),
                          target * 60 / (2 * Math.PI),
                          recoverySeconds,
                          io.peakCurrent,
                          startupPass,
                          steadyPass,
                          recoveryPass,
                          stopPass));
                  if (traceEnabled)
                    Files.writeString(
                        out.resolve(filter.name + "-" + state + "-" + (int) bus + "V.csv"), trace);
                }
              }
            }
          }
        }
      }
    }
    Files.writeString(out.resolve("summary.csv"), summary);
    System.out.print(summary);
  }

  private static void appendTrace(StringBuilder trace, PhysicalIO io, int startMs, double target) {
    trace.append(
        String.format(
            java.util.Locale.ROOT,
            "%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.3f%n",
            (io.ms - startMs) * 0.001,
            io.omega * 60 / (2 * Math.PI),
            io.reported * 60 / (2 * Math.PI),
            target * 60 / (2 * Math.PI),
            io.requestedVolts,
            io.activeVolts,
            io.motorCurrent));
  }
}
