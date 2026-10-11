package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkParameters;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.Test;

/** Physical module mapping is a hardware contract, independent of tuning and optimization. */
class ModulePolarityRegressionTest {
  private static final class ReadableConfig extends SparkFlexConfig {
    Object parameter(SparkParameters parameter) {
      return getParameter(parameter.value);
    }
  }

  @Test
  void calibratedModuleIdsOffsetsAndDrivePolaritiesStayTogether() {
    var modules =
        new com.ctre.phoenix6.swerve.SwerveModuleConstants[] {
          TunerConstants.FrontLeft, TunerConstants.FrontRight,
          TunerConstants.BackLeft, TunerConstants.BackRight
        };
    int[] driveIds = {32, 33, 7, 1};
    int[] steerIds = {6, 5, 52, 25};
    int[] encoderIds = {20, 17, 10, 4};
    boolean[] inverted = {false, true, true, false};
    double[] offsets = {-0.496740004624, 0.000488281250, 0.001436121331, 0.000976562500};
    for (int i = 0; i < modules.length; i++) {
      assertEquals(driveIds[i], modules[i].DriveMotorId, "Drive CAN ID, module " + i);
      assertEquals(steerIds[i], modules[i].SteerMotorId, "Steer CAN ID, module " + i);
      assertEquals(encoderIds[i], modules[i].EncoderId, "CANcoder ID, module " + i);
      assertEquals(inverted[i], modules[i].DriveMotorInverted, "Drive polarity, module " + i);
      assertEquals(offsets[i], modules[i].EncoderOffset, 1e-12, "Straight zero, module " + i);
      assertEquals(true, modules[i].SteerMotorInverted, "Steer polarity, module " + i);
    }
  }

  @Test
  void gainAndDiagnosticRangeChangesReassertBothInversionValuesWithoutTouchingEncoders() {
    for (boolean inverted : new boolean[] {false, true}) {
      for (boolean limits : new boolean[] {false, true}) {
        ReadableConfig drive = new ReadableConfig();
        drive.inverted(!inverted);
        drive.encoder.positionConversionFactor(0.123);
        drive.encoder.velocityConversionFactor(0.456);
        drive.apply(ModuleIOSpark.buildDriveTuningConfig(inverted, 0.005, 0.112, limits));
        assertEquals(inverted, drive.parameter(SparkParameters.kInverted));
        // Encoder settings must survive a partial tuning write, including steering seed units.
        ReadableConfig expected = new ReadableConfig();
        expected.inverted(inverted);
        expected.encoder.positionConversionFactor(0.123);
        expected.encoder.velocityConversionFactor(0.456);
        drive.closedLoop.pid(0, 0, 0);
        drive.closedLoop.feedForward.kV(0);
        expected.closedLoop.pid(0, 0, 0);
        expected.closedLoop.feedForward.kV(0);
        expected.closedLoop.outputRange(limits ? -0.15 : -1, limits ? 0.15 : 1);
        assertEquals(expected.flatten(), drive.flatten());

        ReadableConfig turn = new ReadableConfig();
        turn.inverted(!inverted);
        turn.apply(ModuleIOSpark.buildTurnTuningConfig(inverted, 1.0, 0, limits));
        assertEquals(inverted, turn.parameter(SparkParameters.kInverted));
      }
    }
  }
}
