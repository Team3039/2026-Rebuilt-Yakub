package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * pure math, no hardware
 *
 * Current goal is to prove that {@link ShootingCalculator} produces the correct field frame heading from any
 * robot pose to the hub, matching the same atan2(dy, dx) convention already used by
 * {@code Swerve#getRotationToHub}. 
 * Getting this baseline right is the required for future calculations
 * velocity compensation, interpolation, fire gate 
 * wrong headings here poison everything above it.
 *
 * currently ChassisSpeeds is zero in all cases
 */
class ShootingCalculatorTest {

  // Hub positions must stay in sync with Swerve.BlueHubPose / RedHubPose.
  private static final Translation2d HUB_BLUE = new Translation2d(4.633, 4.030);
  private static final Translation2d HUB_RED  = new Translation2d(11.918, 4.030);

  private static final ChassisSpeeds ZERO_FIELD_VEL = new ChassisSpeeds(0, 0, 0);

  @Test
  void staticAim_robotWestOfHub_turretPointsEast() {
    var calc = new ShootingCalculator(HUB_BLUE);
    var robot = new Pose2d(2.0, 4.030, Rotation2d.fromDegrees(0));

    ShootingSolution s = calc.calculate(robot, ZERO_FIELD_VEL);

    // Pure east vector → atan2(0, +dx) = 0°
    assertEquals(0.0, s.fieldAimToHubDeg(), 1.0);
  }

  @Test
  void staticAim_robotSouthOfHub_turretPointsNorth() {
    var calc = new ShootingCalculator(HUB_BLUE);
    var robot = new Pose2d(4.633, 2.0, Rotation2d.fromDegrees(0));

    ShootingSolution s = calc.calculate(robot, ZERO_FIELD_VEL);

    // Pure north vector → atan2(+dy, 0) = 90°
    assertEquals(90.0, s.fieldAimToHubDeg(), 1.0);
  }

  @Test
  void staticAim_robotNorthOfHub_turretPointsSouth() {
    var calc = new ShootingCalculator(HUB_BLUE);
    var robot = new Pose2d(4.633, 6.0, Rotation2d.fromDegrees(0));

    ShootingSolution s = calc.calculate(robot, ZERO_FIELD_VEL);

    // Pure south vector → atan2(-dy, 0) = -90°
    assertEquals(-90.0, s.fieldAimToHubDeg(), 1.0);
  }

  @Test
  void staticAim_redAlliance_hubStraightAheadOfRobot() {
    var calc = new ShootingCalculator(HUB_RED);
    var robot = new Pose2d(14.0, 4.030, Rotation2d.fromDegrees(180));

    ShootingSolution s = calc.calculate(robot, ZERO_FIELD_VEL);

    // Field bearing to hub is ~180°; robot faces 180°. The turret-relative offset must be ~0°.
    // inputModulus normalises the difference into [-180, 180] to avoid a wrap-around false fail.
    double relativeToHeading =
        MathUtil.inputModulus(
            s.fieldAimToHubDeg() - robot.getRotation().getDegrees(), -180.0, 180.0);
    assertEquals(0.0, relativeToHeading, 1.0);
  }
}
