package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

/**
 * Exercises actual Notifier/DriverStationSim and fake motor IO; never connects to robot hardware.
 */
class IntakeDiagnosticsWatchdogTest {
  private static final String KEY = "IntakeDiagnostics/";

  private static final class BlockingIO implements AngularIO {
    final CountDownLatch enteredRead = new CountDownLatch(1);
    final CountDownLatch resumeRead = new CountDownLatch(1);
    final CountDownLatch stoppedDuringRead = new CountDownLatch(1);
    final AtomicInteger commands = new AtomicInteger();
    volatile boolean block;
    volatile boolean referenceValid;
    volatile double angleDegrees = 95;
    volatile double cap = 1;

    @Override
    public void updateInputs(AngularIOInputs inputs) {
      if (block) {
        enteredRead.countDown();
        try {
          if (!resumeRead.await(2, TimeUnit.SECONDS))
            throw new AssertionError("Blocked read cleanup timed out");
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new AssertionError(e);
        }
      }
      inputs.angle = Degrees.of(angleDegrees);
      inputs.velocity = RadiansPerSecond.of(0);
      inputs.appliedVolts = Volts.of(0);
      inputs.busVolts = Volts.of(12);
      inputs.statorCurrent = Amps.of(0);
      inputs.motorTemperatures = new double[] {25};
      inputs.deviceConnectedStatuses =
          new DeviceConnectedStatus[] {new DeviceConnectedStatus(true, 21)};
      inputs.sampleTimestampSeconds = Timer.getFPGATimestamp();
      inputs.configReady = true;
      inputs.configPending = false;
      inputs.referenceValid = referenceValid;
      inputs.readbackOutputLimit = cap;
      inputs.readbackMinimumAngleRadians = 0;
      inputs.readbackMaximumAngleRadians = Math.toRadians(95);
    }

    @Override
    public boolean diagnosticCalibrateReference(Angle angle) {
      angleDegrees = angle.in(Degrees);
      referenceValid = true;
      return true;
    }

    @Override
    public boolean diagnosticSetOutputLimit(double duty) {
      cap = duty;
      return true;
    }

    @Override
    public void setOpenLoop(Voltage volts) {
      if (volts.in(Volts) != 0) commands.incrementAndGet();
    }

    @Override
    public void setAngle(Angle angle) {
      commands.incrementAndGet();
    }

    @Override
    public void stop() {
      if (block && enteredRead.getCount() == 0) stoppedDuringRead.countDown();
    }
  }

  private static void ds(boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @Test
  void notifierStopsWhileMainInputReadIsBlockedAndResumeCannotReissueOutput() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    ds(false);
    boolean savedTuningMode = Constants.kTuningMode;
    Constants.kTuningMode = false;
    var io = new BlockingIO();
    var subsystem =
        new AngularSubsystem(
            io, AngularSubsystemConfig.builder().logKey("WatchdogTestPivot").build());
    var worker = Executors.newSingleThreadExecutor();
    IntakeDiagnostics diagnostics = null;
    try {
      diagnostics = new IntakeDiagnostics(subsystem);
      SmartDashboard.putBoolean(KEY + "Prepare", true);
      SmartDashboard.putBoolean(KEY + "ReferenceConfirmed", true);
      SmartDashboard.putBoolean(KEY + "ScalingAndTravelConfirmed", true);
      SmartDashboard.putBoolean(KEY + "AllowRemote", true);
      SmartDashboard.putString(KEY + "Mode", "FEEDBACK");
      SmartDashboard.putNumber(KEY + "ReferenceDegrees", 95);
      SmartDashboard.putNumber(KEY + "MinimumDegrees", 92);
      SmartDashboard.putNumber(KEY + "MaximumDegrees", 95);
      SmartDashboard.putNumber(KEY + "Voltage", -.1);
      SmartDashboard.putNumber(KEY + "DurationSeconds", .15);
      SmartDashboard.putNumber(KEY + "CalibrationNonce", 1);
      double now = Timer.getFPGATimestamp();
      diagnostics.periodic(now, 20, false);
      assertTrue(diagnostics.isSelected());
      assertEquals(0, io.commands.get(), "Disabled preparation must not move the pivot");
      ds(true);
      SmartDashboard.putBoolean(KEY + "RemoteActivation", true);
      for (int i = 1; i <= 105; i++) {
        SmartDashboard.putNumber(KEY + "RemoteHeartbeat", i);
        diagnostics.periodic(now + i * .02, 20, false);
      }
      assertTrue(SmartDashboard.getBoolean(KEY + "Ready", false));
      SmartDashboard.putNumber(KEY + "StartNonce", 1);
      SmartDashboard.putNumber(KEY + "RemoteHeartbeat", 106);
      diagnostics.periodic(now + 106 * .02, 20, false);
      assertEquals(
          1, io.commands.get(), "A new eligible nonce should issue one bounded voltage command");
      io.block = true;
      final IntakeDiagnostics selectedDiagnostics = diagnostics;
      var blockedLoop =
          worker.submit(() -> selectedDiagnostics.periodic(now + 107 * .02, 20, false));
      assertTrue(
          io.enteredRead.await(250, TimeUnit.MILLISECONDS), "Fake input poll should be blocked");
      assertTrue(
          io.stoppedDuringRead.await(250, TimeUnit.MILLISECONDS),
          "Independent watchdog must stop while main-thread input poll remains blocked");
      assertFalse(
          blockedLoop.isDone(), "Motor stop must occur before the main-thread poll is released");
      io.resumeRead.countDown();
      blockedLoop.get(1, TimeUnit.SECONDS);
      assertEquals(
          1, io.commands.get(), "A watchdog stop cannot be followed by another nonzero command");
      SmartDashboard.putBoolean(KEY + "StopRequested", true);
      SmartDashboard.putBoolean(KEY + "Prepare", false);
      assertTrue(diagnostics.isSelected(), "Clearing Prepare while enabled must retain isolation");
      ds(false);
      diagnostics.periodic(now + 108 * .02, 20, false);
      assertTrue(diagnostics.isSelected(), "Disabling alone cannot release a requested stop");
      assertEquals(.03, io.cap, 1e-9);
      SmartDashboard.putBoolean(KEY + "StopRequested", false);
      diagnostics.periodic(now + 109 * .02, 20, false);
      assertFalse(diagnostics.isSelected(), "Disabled acknowledged restoration releases isolation");
      assertEquals(1, io.cap, "Normal controller output cap is restored without motion");
      assertEquals(1, io.commands.get());
    } finally {
      io.resumeRead.countDown();
      if (diagnostics != null) diagnostics.close();
      worker.shutdownNow();
      CommandScheduler.getInstance().unregisterSubsystem(subsystem);
      Constants.kTuningMode = savedTuningMode;
      DriverStationSim.resetData();
      DriverStation.refreshData();
    }
  }
}
