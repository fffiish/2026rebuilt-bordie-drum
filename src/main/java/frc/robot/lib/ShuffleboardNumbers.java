package frc.robot.lib;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import java.util.HashMap;
import java.util.Map;

/**
 * Numbers shown on Shuffleboard tabs. Shuffleboard rejects a second widget with the same title, so
 * each entry is created once and reused, even when a subsystem is constructed again (tests, sim).
 */
public final class ShuffleboardNumbers {
  private static final Map<String, GenericEntry> entries = new HashMap<>();

  private ShuffleboardNumbers() {}

  public static void set(String tab, String title, double value) {
    entries
        .computeIfAbsent(
            tab + "/" + title, key -> Shuffleboard.getTab(tab).add(title, 0.0).getEntry())
        .setDouble(value);
  }
}
