package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * pure math shooting calculator: hub aim bearing with velocity compensation extendability
 * frc library {@link frc.robot.subsystems.Swerve#getRotationToHub()} 
 * {@code atan2(hub.y - robot.y, hub.x - robot.x)} in degrees.
 */
public final class ShootingCalculator {

  private final Translation2d hub;

  public ShootingCalculator(Translation2d hubTranslation) {
    this.hub = hubTranslation;
  }

  /**
   * @param robotPose where the robot is on the field and its heading
   * @param fieldRelVelocity field-relative chassis speeds
   */
  public ShootingSolution calculate(Pose2d robotPose, ChassisSpeeds fieldRelVelocity) {
    double dx = hub.getX() - robotPose.getX();
    double dy = hub.getY() - robotPose.getY();
    double fieldAimDeg = Math.toDegrees(Math.atan2(dy, dx));
    double distanceM = Math.hypot(dx, dy);
    double flywheelRps = ShooterRpsTable.lookupRps(distanceM);
    double hoodSetpoint = 0.0;
    boolean readyToFire = false;

    return new ShootingSolution(fieldAimDeg, flywheelRps, hoodSetpoint, readyToFire);
  }

  /**
   * exposes table lookup for tests and for callers that already know distance
   */
  public static double flywheelRpsForDistanceMeters(double distanceMeters) {
    return ShooterRpsTable.lookupRps(distanceMeters);
  }
}
