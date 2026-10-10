package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import org.junit.jupiter.api.Test;

class AngularSubsystemTuningTest {
  private static final class RecordingIO implements AngularIO {
    double[] gains;
    int constraints;
    int stops;
    int angleCommands;
    int capRequests;

    @Override
    public void setGains(double p, double i, double d, double s, double v, double g) {
      gains = new double[] {p, i, d, s, v, g};
    }

    @Override
    public void setConstraints(AngularVelocity velocity, AngularAcceleration acceleration) {
      constraints++;
    }

    @Override
    public void updateInputs(AngularIOInputs inputs) {
      inputs.angle = Degrees.of(30);
      inputs.sampleTimestampSeconds = 123;
      inputs.configReady = true;
      inputs.deviceConnectedStatuses =
          new DeviceConnectedStatus[] {new DeviceConnectedStatus(true, 21)};
    }

    @Override
    public void setAngle(Angle angle) {
      angleCommands++;
    }

    @Override
    public void stop() {
      stops++;
    }

    @Override
    public boolean diagnosticSetOutputLimit(double duty) {
      capRequests++;
      return true;
    }
  }

  @Test
  void diagnosticRefreshForwardsStaticGainPreservesDeviceProfileAndNeverStows() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    DriverStation.refreshData();
    boolean tuningMode = Constants.kTuningMode;
    Constants.kTuningMode = false; // This test never creates or writes NetworkTables tunables.
    RecordingIO io = new RecordingIO();
    AngularSubsystem subsystem = null;
    try {
      var config =
          AngularSubsystemConfig.builder()
              .logKey("StaticGainTest")
              .kP(.01)
              .kS(.45)
              .kV(.07)
              .kG(.2)
              .build();
      subsystem = new AngularSubsystem(io, config);
      subsystem.diagnosticRefresh();
      assertArrayEquals(new double[] {.01, 0, 0, .45, .07, .2}, io.gains);
      assertEquals(0, io.constraints, "Unspecified zero profile must preserve IO limits");
      assertEquals(0, io.angleCommands, "Refresh must not run the stow default");
      assertEquals(123, subsystem.diagnosticTimestampSeconds());
      assertTrue(subsystem.diagnosticConfigReady());
      assertTrue(subsystem.diagnosticConnected());
      assertEquals(30, subsystem.diagnosticAngle().in(Degrees), 1e-9);
      assertTrue(subsystem.diagnosticSetOutputLimit(.1));
      assertFalse(subsystem.diagnosticSetOutputLimit(Double.NaN));
      assertEquals(1, io.capRequests);
      subsystem.diagnosticStop();
      assertEquals(1, io.stops);
      assertEquals(0, io.angleCommands);
    } finally {
      Constants.kTuningMode = tuningMode;
      if (subsystem != null) CommandScheduler.getInstance().unregisterSubsystem(subsystem);
      DriverStationSim.resetData();
      DriverStation.refreshData();
    }
  }
}
