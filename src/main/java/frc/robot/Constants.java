package frc.robot;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class Constants {

    private static final double wheelBase = 0.5; // Define the wheelBase (example value)
    private static final double trackWidth = 0.5; // Define the trackWidth (example value)
    public static final double kDriveGearRatio = 5.357142857142857;
    public static final double wheelCircumference = Units.inchesToMeters(4 * Math.PI);
    public static final double turretGearRatio = 43.16627634660422;
    public static final Transform2d shooterOffset = new Transform2d(new Translation2d(0, 0.1525), new Rotation2d());

    public static final double turretYOffset = Swerve.getPose().getY() + shooterOffset.getTranslation().getY();
    public static final double turretXOffset = Swerve.getPose().getX() + shooterOffset.getTranslation().getX();



    public static final APConstraints kConstraints = new APConstraints()
            .withAcceleration(5.0)
            .withJerk(5);

    public static final APProfile kProfile = new APProfile(kConstraints)
            .withErrorXY(Centimeters.of(1.5))
            .withErrorTheta(Degrees.of(3))
            .withBeelineRadius(Centimeters.of(8));

    public static final Autopilot kAutopilot = new Autopilot(kProfile);

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final class Ports {
        // CLAW
        public static final int INTAKE = 21;
        public static final int INTAKEROLLER = 22;
        public static final int INDEXER = 14;
        public static final int INDEXERCANRANGE = 20;
        public static final int TURRET = 17;
        public static final int FLYWHEEL_LEFT = 18;
        public static final int FLYWHEEL_RIGHT = 15;
        public static final int CANdleID = 13;
        // public static final int INTAKE_ROLLER = 7;
        public static final int HOOD = 16;
        public static final int KICKER = 14;
        public static final int HOPPER = 19;
        public static final int HOPPER2 = 23;

        // KICKER
        // INTAKE_ROLLER
        // TURRET
        // INDEXERCANRANGE
        // dio ports
        public static final int INTAKE_ENCODER = 9;

    }

    public static final class Intake {
        public static final double Intake_KP = 0.065;
        public static final double Intake_KI = 0;
        public static final double Intake_KD = 0.0001;
        public static final double Intake_KS = 0.0005;
        public static final double Intake_KG = 0;
        public static final double Intake_KV = 0;
        public static final double Intake_MAX_VEL = .0;
        public static final double Intake_MAX_ACCEL = .00;
        public static final double Intake_OFFSET = 0;

    }

    public static final class Turret {
        public static final double Turret_KP = 0.0058; // used to be 67 :(
        public static final double Turret_KI = 0;
        public static final double Turret_KD = 0.000;
        public static final double Turret_KS = 0.0007;
        public static final double Turret_KG = 0;
        public static final double Turret_KV = 0;
        public static final double Turret_MAX_VEL = .0;
        public static final double Turret_MAX_ACCEL = .00;
        public static final double Turret_OFFSET = 0;

    }

    public static final class Hood {
        public static final double Hood_KP = 0.03;
        public static final double Hood_KI = 0;
        public static final double Hood_KD = 0.0000;
        public static final double Hood_KS = 0.05;
        public static final double Hood_KG = 0;
        public static final double Hood_KV = 0;
        public static final double Hood_MAX_VEL = .4;
        public static final double Hood_MAX_ACCEL = .20;
        public static final double Hood_OFFSET = 0;
    }

    public static final class Flywheel {
        public static final double Flywheel_KP = 0.12;
        public static final double Flywheel_KI = 0.000;
        public static final double Flywheel_KD = 0.000;
        public static final double Flywheel_FF = 1.75;

    }

}
