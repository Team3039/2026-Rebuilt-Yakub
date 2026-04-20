package frc.robot.subsystems;

import java.util.function.Supplier;

//import com.ctre.phoenix6.SignalLogger;
// import frc.robot.Utilitys;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
// import frc.robot.Utilitys;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {

    public static final Pose2d BlueHubPose = new Pose2d(4.633, 4.030, Rotation2d.fromDegrees(0));

    public static final Pose2d RedHubPose = new Pose2d(11.918, 4.030, Rotation2d.fromDegrees(0));


     
       
    
    
        static double targetYaw;
    
        public SwerveDrivePoseEstimator m_poseEstimator;
        public LimelightHelpers.PoseEstimate mt2;
        public LimelightHelpers.PoseEstimate leftPose;
        public LimelightHelpers.PoseEstimate rightPose;
        public LimelightHelpers.PoseEstimate[] cameraPoses = new LimelightHelpers.PoseEstimate[2];
    
        public Pigeon2 gyro;
        // public SwerveDriveOdometry swerveOdometry;
        public PoseEstimate cameraPose;
        // public Pose2d botPose = new Pose2d();
        public static Pose2d botPose2d = new Pose2d();
        public Pose3d botPose3d = new Pose3d();
        public PoseEstimate best = new PoseEstimate();
        // public Utilitys utils = new Utilitys();
        // private final SwerveModule[] swerveModules = new SwerveModule[] { new
        // SwerveModule(0, 1), new SwerveModule(2, 3),
        // new SwerveModule(4, 5), new SwerveModule(6, 7) };
    
        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
        /* Keep track if we've ever applied the operator perspective before or not */
        private boolean m_hasAppliedOperatorPerspective = false;
    
        /** Swerve request to apply during robot-centric path following */
        private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    
        /* Swerve requests to apply during SysId characterization */
        // private final SwerveRequest.SysIdSwerveTranslation
        // m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
        // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
        // new SwerveRequest.SysIdSwerveSteerGains();
        // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
        // new SwerveRequest.SysIdSwerveRotation();
    
        /*
         * SysId routine for characterizing translation. This is used to find PID gains
         * for the drive motors.
         */
    
        public Swerve(SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(drivetrainConstants, modules);
            gyro = new Pigeon2(0, "Sweeve Modules");
            // gyro.setYaw(0);
    
            // SmartDashboard.putNumber("yaw2", gyro.getYaw().getValueAsDouble());
    
            getModule(0).getDriveMotor().setPosition(0);
            getModule(1).getDriveMotor().setPosition(0);
            getModule(2).getDriveMotor().setPosition(0);
            getModule(3).getDriveMotor().setPosition(0);
    



            m_poseEstimator = new SwerveDrivePoseEstimator(Constants.swerveKinematics, getGyroRotation2D(),
                    getModulePositions(), getPose(), VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(.1)),
                    VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(.5)));
    



            configureAutoBuilder();

            
        }
    
        /*
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * 
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         * unspecified or set to 0 Hz, this is 250 Hz on
         * CAN FD, and 100 Hz on CAN 2.0.
         * 
         * @param modules Constants for each specific module
         */
    
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve
         *                                  drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz
         *                                  on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
    
        public void configureAutoBuilder() {
            try {
                var config = RobotConfig.fromGUISettings();
                AutoBuilder.configure(
                        () -> getPose(), /// getState().Pose, // Supplier of current robot pose
                        this::resetOdometry, // Consumer for seeding pose against auto
                        () -> getState().Speeds, // Supplier of current robot speeds
                        // Consumer of ChassisSpeeds and feedforwards to drive the robot
                        (speeds, feedforwards) -> setControl(
                                m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                        new PPHolonomicDriveController(
                                // PID constants for translation
                                new PIDConstants(2., 0, 0), // was 0.5 //was 0.7
                                // kP10
                                // PID constants for rotation
                                new PIDConstants(2, 0, 0)), // was 2.0
                        config,
                        // Assume the path needs to be flipped for Red vs Blue, this is normally the
                        // case
                        () -> {
                            var alliance = DriverStation.getAlliance();
                            if (alliance.isPresent()) {
                                return alliance.get() == DriverStation.Alliance.Red;
    
                            }
                            return false;
                        },
                        this // Subsystem for requirements
                );
            } catch (Exception ex) {
                DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                        ex.getStackTrace());
            }
        }
    
        /**
         * Returns a command that applies the specified control request to this swerve
         * drivetrain.
         *
         * @param request Function returning the request to apply
         * @return Command to run
         */
        public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
            return run(() -> this.setControl(requestSupplier.get()));
        }
    
        /**
         * Runs the SysId Quasistatic test in the given direction for the routine
         * specified by {@link #m_sysIdRoutineToApply}.
         *
         * @param direction Direction of the SysId Quasistatic test
         * @return Command to run
         */
    
        public void updateCameraPose() {
            boolean doRejectUpdate = false;
            int bestCamera;
            double leftAmbiguity = 0;
            double rightAmbiguity = 0;
    
            cameraPoses[1] = grabPose("limelight-front");
            cameraPoses[0] = grabPose("limelight-back");
    
            if (cameraPoses[0] == null && cameraPoses[1] == null) {
                bestCamera = -1;
            } else if (cameraPoses[0] == null) {
                bestCamera = 1;
            } else if (cameraPoses[1] == null) {
                bestCamera = 0;
            } else {
                if (cameraPoses[0].tagCount > 0) {
                    leftAmbiguity = cameraPoses[0].rawFiducials[0].ambiguity;
                }
                if (cameraPoses[1].tagCount > 0) {
                    rightAmbiguity = cameraPoses[1].rawFiducials[0].ambiguity;
                }
                if (leftAmbiguity < rightAmbiguity) {
                    bestCamera = 0;
                } else {
                    bestCamera = 1;
                }
            }
            if (bestCamera == -1) {
                doRejectUpdate = true;
            } else {
                if (cameraPoses[bestCamera].tagCount < 1) {
                    doRejectUpdate = true;
                }
            }
    
            ;
    
            if (gyro.getAngularVelocityZWorld().getValueAsDouble() > 360) // if our angular velocity is greater
            {
                doRejectUpdate = true;
            }
    
            double maxTagDistance = 4;
    
            if (!doRejectUpdate) {
                double dist = cameraPoses[bestCamera].rawFiducials[0].distToCamera;
    
                if (dist > maxTagDistance) {
                    doRejectUpdate = true;
                }
    
                SmartDashboard.putBoolean("RejectUpdate", doRejectUpdate);
                if (!doRejectUpdate) {
                    SmartDashboard.putNumber("bestcamera", bestCamera);
                    SmartDashboard.putNumberArray("CameraPose",
                            new double[] { cameraPoses[bestCamera].pose.getTranslation().getX(),
                                    cameraPoses[bestCamera].pose.getTranslation().getY(),
                                    cameraPoses[bestCamera].pose.getRotation().getRadians() });
                    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999));
                    m_poseEstimator.addVisionMeasurement(
                            cameraPoses[bestCamera].pose,
                            cameraPoses[bestCamera].timestampSeconds);
    
                }
            }
        }
    
        private static Pose2d getHubPoseForAlliance() {
            var allianceOpt = DriverStation.getAlliance();
            if (allianceOpt.isPresent()) {
                return allianceOpt.get() == Alliance.Red ? RedHubPose : BlueHubPose;
            }
    
            return BlueHubPose;
        }
    
        

             public  double getRotationToHub() {

                         Pose2d shooterPositionPose = getPose2d().plus(Constants.shooterOffset);

                    Pose2d hub = getHubPoseForAlliance();
            
                    targetYaw = Math.atan2(
                    hub.getY() - shooterPositionPose.getY() ,
                    hub.getX() - shooterPositionPose.getX());

        return Math.toDegrees(targetYaw);
    }

    public double getDistanceToHub() {

         Pose2d shooterPositionPose = getPose2d().plus(Constants.shooterOffset);


        Pose2d hub = getHubPoseForAlliance();
        return Math.hypot(
                hub.getX() -   shooterPositionPose.getY() ,
                hub.getY() -   shooterPositionPose.getX() );
    }
            
              



    @Override
    public void periodic() {



                  Pose2d shooterPositionPose = getPose2d().plus(Constants.shooterOffset);



       

        m_poseEstimator.update(getGyroRotation2D(), getModulePositions());
        botPose2d = m_poseEstimator.getEstimatedPosition();
        // SmartDashboard.putNumber("Rotation2D",getGyroRotation2D().getDegrees());

        updateCameraPose();

        
        
        SmartDashboard.putNumberArray("bot Pose",
        
                new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getRadians() });

        SmartDashboard.putNumber("yaw", gyro.getRotation2d().getRadians());

        SmartDashboard.putNumber("getDistanceToHub", getDistanceToHub());

 SmartDashboard.putNumberArray("Turret Pose", 
        new double[] {  shooterPositionPose.getX(), shooterPositionPose.getY(), RobotContainer.turret.getTurretPosition()*-1 });

    }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroRotation2D(),
                getModulePositions(), pose);
    }

    public Rotation2d getGyroscopeRotation() {

        return Rotation2d.fromDegrees(getCompassHeading());// gyro.getYaw().getValueAsDouble());
    }

    public void resetGyro() {
        gyro.setYaw(0);
    }

    public void resetGyro(double heading) {
        gyro.setYaw(heading);
    }

    public void resetGyroToAlliance() {
        gyro.setYaw(DriverStation.getAlliance().get() == Alliance.Red ? 0 : 180);
    }

    public static Pose2d getPose() {
        return botPose2d;
    }
    

    public Pose2d getPose2d() {
        return new Pose2d(getPose().getTranslation(), getPose().getRotation());
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {

        getCompassHeading();
        m_poseEstimator.resetPosition(getGyroRotation2D(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public double getCompassHeading() {
        SmartDashboard.putNumber("CompassHeading", Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360.0);
    }

    
    

    public Rotation2d getGyroRotation2D() {
        return Rotation2d.fromDegrees(getCompassHeading());// gyro.getYaw().getValueAsDouble());
    }

    public PoseEstimate grabPose(String camera) {
        // LimelightHelpers.SetRobotOrientation( "limelight-front",
        // gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);

        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
        return mt2;

    }


     

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
    // Optional<PoseEstimate> leftEstimate = left.update(swerve.getMegaTag2Yaw());
    // Optional<PoseEstimate> rightEstimate = right.update(swerve.getMegaTag2Yaw());

    public SwerveModulePosition[] getModulePositions() {

        /*
         * AI\\
         * SwerveModulePosition[] positions = new
         * SwerveModulePosition[swerveModules.length];
         * for (int i = 0; i < swerveModules.length; i++) {
         * positions[i] = swerveModules[i].getPosition();
         * }
         * return positions;
         */

        // Return the positions of the swerve modules

        return new SwerveModulePosition[] {
                // Replace with actual module positions
                new SwerveModulePosition(getModule(0).getDriveMotor().getPosition().getValueAsDouble()
                        * Constants.wheelCircumference / Constants.kDriveGearRatio,
                        getModule(0).getCurrentState().angle),
                new SwerveModulePosition(getModule(1).getDriveMotor().getPosition().getValueAsDouble()
                        * Constants.wheelCircumference / Constants.kDriveGearRatio,
                        getModule(1).getCurrentState().angle),
                new SwerveModulePosition(getModule(2).getDriveMotor().getPosition().getValueAsDouble()
                        * Constants.wheelCircumference / Constants.kDriveGearRatio,
                        getModule(2).getCurrentState().angle),
                new SwerveModulePosition(getModule(3).getDriveMotor().getPosition().getValueAsDouble()
                        * Constants.wheelCircumference / Constants.kDriveGearRatio,
                        getModule(3).getCurrentState().angle)
        };

    }
}