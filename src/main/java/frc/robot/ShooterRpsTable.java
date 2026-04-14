package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Distance (m, robot -> hub) → flywheel magnitude (RPS). Single source of truth for TRACKING so
 * {@link ShootingCalculator} and {@link frc.robot.subsystems.Flywheel} stay aligned.
 */
public final class ShooterRpsTable {

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_RPS = new InterpolatingDoubleTreeMap();

  static {
    // Copied from Flywheel dissierdShooterSpeedv2 — tune on robot
    DISTANCE_TO_RPS.put(1.80, 4.44);
    DISTANCE_TO_RPS.put(1.70, 4.43);
    DISTANCE_TO_RPS.put(1.60, 4.31);
    DISTANCE_TO_RPS.put(2.9, 4.4);
    DISTANCE_TO_RPS.put(3.1, 5.0);
    DISTANCE_TO_RPS.put(4.0, 5.2);
    DISTANCE_TO_RPS.put(5.6, 5.7);
    DISTANCE_TO_RPS.put(5.9, 6.3);
  }

  private ShooterRpsTable() {}

  /** WPILib interpolates between keys */
  public static double lookupRps(double distanceMeters) {
    return DISTANCE_TO_RPS.get(distanceMeters);
  }
}
