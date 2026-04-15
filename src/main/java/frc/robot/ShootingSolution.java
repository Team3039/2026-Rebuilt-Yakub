package frc.robot;

/**
 * Output of {@link ShootingCalculator}. {@link #fieldAimToHubDeg()}
 * other fields are stubs until later.
 */
public record ShootingSolution(
    double fieldAimToHubDeg,
    double flywheelRps,
    double hoodSetpoint,
    boolean readyToFire) {}
