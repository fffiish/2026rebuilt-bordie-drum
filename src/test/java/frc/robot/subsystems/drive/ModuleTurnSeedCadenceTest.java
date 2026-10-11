package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ModuleTurnSeedCadenceTest {
  @Test
  void failedBootAndDisabledAttemptsWaitOneSecondBeforeRetrying() {
    var cadence = new ModuleIOSpark.TurnSeedCadence(new ModuleIOSpark.SeedAttemptBudget());
    cadence.recordAttempt(0.0);
    assertFalse(cadence.acquire(0.999, true, true, true, false, 0.0));
    assertTrue(cadence.acquire(1.0, true, true, true, false, 0.0));
    // A failed hardware write leaves seeded false, but still consumes its retry interval.
    assertFalse(cadence.acquire(1.02, true, true, true, false, 0.0));
    assertFalse(cadence.acquire(1.999, true, true, true, false, 0.0));
    assertTrue(cadence.acquire(2.0, true, true, true, false, 0.0));
  }

  @Test
  void enabledUnconfiguredMissingAndMovingControllersDoNotConsumeTheBudget() {
    var cadence = new ModuleIOSpark.TurnSeedCadence(new ModuleIOSpark.SeedAttemptBudget());
    assertFalse(cadence.acquire(1.0, false, true, true, true, 0.0));
    assertFalse(cadence.acquire(1.0, true, false, true, true, 0.0));
    assertFalse(cadence.acquire(1.0, true, true, false, true, 0.0));
    assertFalse(cadence.acquire(1.0, true, true, true, true, 0.1));
    assertFalse(cadence.acquire(1.0, true, true, true, true, Double.NaN));
    assertFalse(cadence.acquire(Double.NaN, true, true, true, true, 0.0));
    assertTrue(cadence.acquire(1.0, true, true, true, true, 0.0));
  }

  @Test
  void missingControllerCannotDelayAHealthyModulesRetry() {
    var budget = new ModuleIOSpark.SeedAttemptBudget();
    var missing = new ModuleIOSpark.TurnSeedCadence(budget);
    var healthy = new ModuleIOSpark.TurnSeedCadence(budget);
    for (int loop = 0; loop < 50; loop++) {
      assertFalse(missing.acquire(loop * 0.020, true, true, false, false, 0.0));
    }
    assertTrue(healthy.acquire(1.0, true, true, true, false, 0.0));
  }

  @Test
  void sharedBudgetSpreadsAllFourModulesAcrossLoopsWithoutStarvation() {
    var budget = new ModuleIOSpark.SeedAttemptBudget();
    var modules = new ModuleIOSpark.TurnSeedCadence[4];
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new ModuleIOSpark.TurnSeedCadence(budget);
    }
    int[] attempts = new int[4];
    for (int loop = 0; loop < 4; loop++) {
      int attemptsThisLoop = 0;
      for (int i = 0; i < modules.length; i++) {
        if (modules[i].acquire(1.0 + loop * 0.025, true, true, true, false, 0.0)) {
          attempts[i]++;
          attemptsThisLoop++;
        }
      }
      assertEquals(1, attemptsThisLoop);
    }
    for (int attemptCount : attempts) {
      assertEquals(1, attemptCount);
    }
  }
}
