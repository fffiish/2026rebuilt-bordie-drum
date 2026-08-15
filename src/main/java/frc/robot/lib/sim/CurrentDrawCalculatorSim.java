package frc.robot.lib.sim;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class CurrentDrawCalculatorSim extends VirtualSubsystem {
  private final List<Supplier<Current>> subsystemCurrentDraws = new ArrayList<>();

  double batteryVoltage = 12.0f;

  @SafeVarargs
  public final void registerCurrentDraw(Supplier<Current>... voltageDraws) {
    Collections.addAll(subsystemCurrentDraws, voltageDraws);
  }

  public void periodic() {
    if (!Constants.currentMode.equals(Constants.Mode.SIM)) return;

    double[] draws = new double[subsystemCurrentDraws.size()];
    for (int i = 0; i < subsystemCurrentDraws.size(); i++) {
      draws[i] = subsystemCurrentDraws.get(i).get().in(Amps);
    }
    double newVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(draws);
    batteryVoltage = batteryVoltage * 0.95 + newVoltage * 0.05; // Simple low-pass filter

    RoboRioSim.setVInVoltage(batteryVoltage);

    Logger.recordOutput(
        "CurrentDrawCalculatorSim/BatteryVoltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("CurrentDrawCalculatorSim/CurrentDraws", draws);
  }
}
