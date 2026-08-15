package frc.robot.constants;

import frc.robot.lib.LoggedTunableNumber;

public class DriveConstants {

  // p gain for the drive x and y controllers (side to side and forward/back)
  public static LoggedTunableNumber TRANSLATION_KP =
      new LoggedTunableNumber("Drive/TranslationKp", 3.0);
  public static LoggedTunableNumber TRANSLATION_KD =
      new LoggedTunableNumber("Drive/TranslationKd", 0.0);
  public static LoggedTunableNumber PP_ANGLE_KP = new LoggedTunableNumber("Drive/PPAngleKp", 1.5);
  public static LoggedTunableNumber PP_ANGLE_KD = new LoggedTunableNumber("Drive/PPAngleKd", 0.0);

  // p gain for the drive x and y controllers (side to side and forward/back)
  public static LoggedTunableNumber TRENCH_TRANSLATION_KP =
      new LoggedTunableNumber("Drive/Trench/TranslationKp", 8.0);
  public static LoggedTunableNumber TRENCH_TRANSLATION_KD =
      new LoggedTunableNumber("Drive/Trench/TranslationKd", 0.0);
  public static LoggedTunableNumber TRENCH_ANGLE_KP =
      new LoggedTunableNumber("Drive/Trench/AngleKp", 4.0);
  public static LoggedTunableNumber TRENCH_ANGLE_KD =
      new LoggedTunableNumber("Drive/Trench/AngleKd", 0.0);

  public static final double DEADBAND = 0.05;

  // p gain for the drive angle controller;
  public static final double ANGLE_MAX_VELOCITY = 8.0; // Rad per sec
  public static final double ANGLE_MAX_ACCELERATION = 20.0; // Rad per sec^2
  public static final double TRANSLATION_MAX_VELOCITY = 3.0; // Meters per sec
  public static final double TRANSLATION_MAX_ACCELERATION = 5.0; // Meters per sec^2
  public static final double FF_START_DELAY = 2.0; // Secs
  public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  public static final double RIGHT_TRENCH_Y = 0.625;
  public static final double LEFT_TRENCH_Y = 7.425;

  // DT speed config

  public static final double INTAKE_MULT = 1.0;
  public static final double INTAKE_MULT_W = 1.0;

  public static LoggedTunableNumber TRANSFER_SPEED = new LoggedTunableNumber("Drive/SOTM", 0.6);
  public static final double TRANSFER_SPEED_W = 0.55 * TRANSFER_SPEED.get();

  public static LoggedTunableNumber TRANSFER_SPEED_NEUTRAL =
      new LoggedTunableNumber("Drive/SOTM_NEUTRAL", 0.8);
  public static final double TRANSFER_SPEED_W_NEUTRAL = 0.55 * TRANSFER_SPEED_NEUTRAL.get();

  public static LoggedTunableNumber MAX_SPEED = new LoggedTunableNumber("Drive/MAX", 1);
  public static final double MAX_SPEED_W = 1;
}
