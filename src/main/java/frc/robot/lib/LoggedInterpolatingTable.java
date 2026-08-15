package frc.robot.lib;

import frc.robot.Constants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for an interpolated lookup table. Gets values from dashboard in tuning mode, returns
 * default if not or value not in dashboard. Supports linear interpolation between data points.
 * Automatically updates via a static registry - no need to call periodic manually.
 */
@SuppressWarnings("unused")
public class LoggedInterpolatingTable {
  private static final List<LoggedInterpolatingTable> instances = new ArrayList<>();
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private TreeMap<Double, Double> defaultTable = new TreeMap<>();
  private LoggedNetworkNumber entryCountNumber;
  private Map<Integer, LoggedNetworkNumber> inputNumbers = new HashMap<>();
  private Map<Integer, LoggedNetworkNumber> outputNumbers = new HashMap<>();
  private Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();
  private int lastEntryCount = 0;

  /**
   * Create a new LoggedInterpolatingTable
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedInterpolatingTable(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
    instances.add(this);
  }

  /**
   * Create a new LoggedInterpolatingTable with default values
   *
   * @param dashboardKey Key on dashboard
   * @param defaultData Nx2 matrix where each row is [input, output]
   */
  public LoggedInterpolatingTable(String dashboardKey, double[][] defaultData) {
    this(dashboardKey);
    initDefault(defaultData);
  }

  /**
   * Set the default values of the table. The default values can only be set once.
   *
   * @param defaultData Nx2 matrix where each row is [input, output]
   */
  public void initDefault(double[][] defaultData) {
    if (!hasDefault) {
      hasDefault = true;

      // Populate default table
      for (double[] entry : defaultData) {
        if (entry.length >= 2) {
          defaultTable.put(entry[0], entry[1]);
        }
      }

      if (Constants.kTuningMode) {
        // Create network table entries
        entryCountNumber = new LoggedNetworkNumber(key + "/EntryCount", defaultTable.size());
        lastEntryCount = defaultTable.size();

        int index = 0;
        for (Map.Entry<Double, Double> entry : defaultTable.entrySet()) {
          inputNumbers.put(
              index, new LoggedNetworkNumber(key + "/Entry" + index + "/Input", entry.getKey()));
          outputNumbers.put(
              index, new LoggedNetworkNumber(key + "/Entry" + index + "/Output", entry.getValue()));
          index++;
        }
      }
    }
  }

  /**
   * Get the interpolated output value for the given input
   *
   * @param input The input value to look up
   * @return The interpolated output value
   */
  public double get(double input) {
    if (!hasDefault) {
      return 0.0;
    }

    TreeMap<Double, Double> table = Constants.kTuningMode ? getTableFromDashboard() : defaultTable;

    if (table.isEmpty()) {
      return 0.0;
    }

    // Handle edge cases
    if (input <= table.firstKey()) {
      return table.firstEntry().getValue();
    }
    if (input >= table.lastKey()) {
      return table.lastEntry().getValue();
    }

    // Find surrounding entries for interpolation
    Map.Entry<Double, Double> lowerEntry = table.floorEntry(input);
    Map.Entry<Double, Double> upperEntry = table.ceilingEntry(input);

    if (lowerEntry == null) {
      return upperEntry.getValue();
    }
    if (upperEntry == null) {
      return lowerEntry.getValue();
    }

    // Linear interpolation
    double x0 = lowerEntry.getKey();
    double y0 = lowerEntry.getValue();
    double x1 = upperEntry.getKey();
    double y1 = upperEntry.getValue();

    if (x0 == x1) {
      return y0;
    }

    return y0 + (y1 - y0) * (input - x0) / (x1 - x0);
  }

  /**
   * Periodic callback that ensures values are continuously read and logged to AdvantageScope.
   * Called automatically via the static updater - no need to call manually.
   */
  private void periodic() {
    if (Constants.kTuningMode && hasDefault) {
      // Trigger a read to ensure all values are logged
      getTableFromDashboard();
    }
  }

  /**
   * Updates all registered LoggedInterpolatingTable instances. Call this once per robot periodic
   * (e.g., from Robot.java or a dedicated virtual subsystem).
   */
  public static void periodicAll() {
    for (LoggedInterpolatingTable table : instances) {
      table.periodic();
    }
  }

  /**
   * Read the current table from NetworkTables and handle entry cleanup
   *
   * @return TreeMap representing the current lookup table
   */
  private TreeMap<Double, Double> getTableFromDashboard() {
    TreeMap<Double, Double> table = new TreeMap<>();

    // Always read the entry count to ensure it's logged
    int currentEntryCount = (int) entryCountNumber.get();

    // If entry count decreased, remove excess entries from NetworkTables
    if (currentEntryCount < lastEntryCount) {
      for (int i = currentEntryCount; i < lastEntryCount; i++) {
        // Remove the network table entries
        inputNumbers.remove(i);
        outputNumbers.remove(i);
      }
    }

    // If entry count increased, create new entries
    if (currentEntryCount > lastEntryCount) {
      for (int i = lastEntryCount; i < currentEntryCount; i++) {
        inputNumbers.put(i, new LoggedNetworkNumber(key + "/Entry" + i + "/Input", 0.0));
        outputNumbers.put(i, new LoggedNetworkNumber(key + "/Entry" + i + "/Output", 0.0));
      }
    }

    lastEntryCount = currentEntryCount;

    // Read all current entries (this ensures they get logged)
    for (int i = 0; i < currentEntryCount; i++) {
      if (inputNumbers.containsKey(i) && outputNumbers.containsKey(i)) {
        double input = inputNumbers.get(i).get();
        double output = outputNumbers.get(i).get();
        table.put(input, output);
      }
    }

    return table;
  }

  /**
   * Checks whether the table has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the table has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    if (!Constants.kTuningMode) {
      // In non-tuning mode, table never changes
      Boolean lastValue = lastHasChangedValues.get(id);
      if (lastValue == null) {
        lastHasChangedValues.put(id, true);
        return true;
      }
      return false;
    }

    // Check if entry count changed
    int currentEntryCount = (int) entryCountNumber.get();
    if (currentEntryCount != lastEntryCount) {
      lastHasChangedValues.put(id, true);
      return true;
    }

    // Check if any input or output values changed
    for (int i = 0; i < currentEntryCount; i++) {
      if (inputNumbers.containsKey(i) && outputNumbers.containsKey(i)) {
        // This will trigger change detection by reading values
        inputNumbers.get(i).get();
        outputNumbers.get(i).get();
      }
    }

    // For simplicity, we'll mark as changed if we haven't checked before
    Boolean lastValue = lastHasChangedValues.get(id);
    if (lastValue == null) {
      lastHasChangedValues.put(id, true);
      return true;
    }

    // In a full implementation, you'd track individual value changes
    // For now, return false if we've checked before (simplified)
    return false;
  }

  /**
   * Get the current lookup table as a 2D array
   *
   * @return Nx2 array where each row is [input, output]
   */
  public double[][] getTable() {
    TreeMap<Double, Double> table = Constants.kTuningMode ? getTableFromDashboard() : defaultTable;

    double[][] result = new double[table.size()][2];
    int index = 0;
    for (Map.Entry<Double, Double> entry : table.entrySet()) {
      result[index][0] = entry.getKey();
      result[index][1] = entry.getValue();
      index++;
    }
    return result;
  }

  /**
   * Get the minimum key (input) in the lookup table.
   *
   * @return minimum key value, or 0.0 if the table is not initialized or empty
   */
  public double getMinKey() {
    if (!hasDefault) {
      return 0.0;
    }
    TreeMap<Double, Double> table = Constants.kTuningMode ? getTableFromDashboard() : defaultTable;
    if (table.isEmpty()) {
      return 0.0;
    }
    return table.firstKey();
  }

  /**
   * Get the maximum key (input) in the lookup table.
   *
   * @return maximum key value, or 0.0 if the table is not initialized or empty
   */
  public double getMaxKey() {
    if (!hasDefault) {
      return 0.0;
    }
    TreeMap<Double, Double> table = Constants.kTuningMode ? getTableFromDashboard() : defaultTable;
    if (table.isEmpty()) {
      return 0.0;
    }
    return table.lastKey();
  }

  /**
   * Get the number of entries in the table
   *
   * @return Number of entries
   */
  public int size() {
    if (!hasDefault) {
      return 0;
    }
    return Constants.kTuningMode ? (int) entryCountNumber.get() : defaultTable.size();
  }
}
