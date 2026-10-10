package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.constants.intake.IntakeConstants;
import org.junit.jupiter.api.Test;

class IntakeReferenceUnitsTest {
  @Test
  void confirmedStowedReferenceEqualsConfiguredLimitExactly() {
    assertEquals(118.0, IntakeConstants.kPivotStowed.in(Degrees), 1e-12);
    assertEquals(
        IntakeConstants.kPivotStowed.in(Radians), IntakeDiagnostics.referenceRadians(118.0));
    // Math.toRadians differs by one representable step at 118 degrees. The IO
    // correctly rejects an actual angle beyond its configured physical limit.
    assertTrue(Math.toRadians(118.0) > IntakeConstants.kPivotStowed.in(Radians));
  }
}
