// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
// import edu.wpi.first.units.Units;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Turret;
import frc.robot.commands.ActuateHoodToSetpoint;
import frc.robot.commands.setFlyWheels;
import frc.robot.commands.setHoodManual;
import frc.robot.commands.setIntakeManual;
import frc.robot.commands.setIntakePassiveUp;
import frc.robot.commands.setIntakeStop;
import frc.robot.commands.setIntakerollersIntake;
import frc.robot.commands.setIntakerollersStop;
import frc.robot.commands.setKickerBackPassive;
import frc.robot.commands.setKickerPassive;
import frc.robot.commands.setTurretIdle;
import frc.robot.commands.setTurretTracking;
import frc.robot.commands.movementCommands.IntakeIdle;
import frc.robot.commands.movementCommands.IntakeIntakeing;
import frc.robot.commands.movementCommands.IntakeZero;
import frc.robot.commands.movementCommands.turretToZero;
import frc.robot.commands.movementCommands.TestShoot;
import frc.robot.commands.movementCommands.hoodToPoint;
import frc.robot.commands.movementCommands.hoodToZero;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRoller;


public class RobotContainer {

        private final SendableChooser<Command> autoChooser;



        public RobotContainer() {

                NamedCommands.registerCommand("Depo side mid run start", drivetrain.runOnce(
                          () -> drivetrain.resetPose(new Pose2d(4.440, 7.582, Rotation2d.fromDegrees(180.000)))));

//                 guitar.a().onTrue(new IntakeIntakeing());
//                 guitar.b().onTrue(new IntakeIdle());
//                 guitar.povDown().whileTrue(new setFlyWheels());
//                 guitar.y().whileTrue(new setTurretTracking());



                NamedCommands.registerCommand("Start Intake", new IntakeIntakeing());
                NamedCommands.registerCommand("Intake back in", new IntakeIdle());
                NamedCommands.registerCommand("Intake slow in", new setIntakePassiveUp());
                NamedCommands.registerCommand("AIM", new setTurretTracking());
                NamedCommands.registerCommand("FIRE!!!!", new setFlyWheels());
                NamedCommands.registerCommand("fix auto for red stuff",  (drivetrain.runOnce(() -> drivetrain.seedFieldCentric())));



                // new setIntakePassiveUp()
// (drivetrain.runOnce(() -> drivetrain.seedFieldCentric()))

                autoChooser = AutoBuilder.buildAutoChooser(); // Auto chooser
                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureBindings();

        }

        public final static CommandXboxController driverPad = new CommandXboxController(0);
        public final static CommandXboxController guitar = new CommandXboxController(1);

        public final Swerve drivetrain = TunerConstants.createDrivetrain();
        public static final Turret turret = new Turret();
        public static final Hood hood = new Hood();
        public static final Flywheel flywheel = new Flywheel();
        public static final Indexer indexer = new Indexer();
        public static final Hopper hopper = new Hopper();
        public static final Intake intake = new Intake();
        public static final IntakeRoller IntakeRoller = new IntakeRoller();



        
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        /* Path follower */   
        // private final SendableChooser<Command> autoChooser;

        private void configureBindings() {

                // Drivetrain
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                // Driver pad

           drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-driverPad.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driverPad.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverPad.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


                // driver controls
                driverPad.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                

//                            SmartDashboard.putNumberArray("bot Pose", new double[] {getPose().getX(), getPose().getY(), getPose().getRotation().getRadians()});



        // driverPad.y().onTrue (drivetrain.runOnce(  () -> drivetrain.resetOdometry(new Pose2d(1.911, 4.030, Rotation2d.fromDegrees(0)))));

            driverPad.a().whileTrue(new setIntakePassiveUp()); 



                driverPad.y().onTrue(
                        drivetrain.runOnce(() -> {
                                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                                        drivetrain.resetOdometry(new Pose2d(2.405, 4.030, Rotation2d.fromDegrees(0))); // blue pose
                                } else {
                                        drivetrain.resetOdometry(new Pose2d(13.934, 4.030, Rotation2d.fromDegrees(180))); // red pose
                                }
                        })
                );

                driverPad.x().onTrue(
                        drivetrain.runOnce(() -> {
                                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                                        drivetrain.resetOdometry(new Pose2d(2.173, 6.040, Rotation2d.fromDegrees(0))); // blue pose
                                } else {
                                        drivetrain.resetOdometry(new Pose2d(14.327, 2.241, Rotation2d.fromDegrees(180))); // red pose
                                }
                        })
                );

                driverPad.b().onTrue(
                        drivetrain.runOnce(() -> {
                                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                                        drivetrain.resetOdometry(new Pose2d(2.173, 2.019, Rotation2d.fromDegrees(0))); // blue pose
                                } else {
                                        drivetrain.resetOdometry(new Pose2d(14.327, 6.413, Rotation2d.fromDegrees(180))); // red pose
                                }
                        })
                );


                // driverPad.a().whileTrue(new setFlyWheels());
                // driverPad.a().onFalse(new setTurretIdle());

                // driverPad.b().onTrue(new IntakeIdle());
                // driverPad.a().whileTrue(new setTurretTracking());
  
                // driverPad.x().whileTrue(new setIntakeManual());

                // guitar.povDown().whileTrue(new setFlyWheels());
                // driverPad.a().onFalse(new setIntakeStop());

                
                guitar.a().onTrue(new IntakeIntakeing()); // the green button
                guitar.b().onTrue(new IntakeIdle()); // the red button
                guitar.y().whileTrue(new setIntakePassiveUp()); // the yellow button
                guitar.x().whileTrue(new setTurretTracking()); // the blue button
                guitar.leftBumper().onTrue(new setIntakerollersIntake());


                guitar.povDown().whileTrue(new setFlyWheels()); // down on the strum bar
                guitar.povUp().whileTrue(new setKickerBackPassive()); // up on the strum bar


                //  driverPad.b().whileTrue(drivetrain.pointAtHubCommand(() -> -driverPad.getLeftY() * MaxSpeed, () -> -driverPad.getLeftX() * MaxSpeed));
                // driverPad.b().onFalse(new setTurretIdle());




                // driverPad.a().whileTrue(new TestShoot());

                // co driver controls, and yes it is a guitar hero controller
                // guitar.y().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(1.567, 3.761, Rotation2d.fromDegrees(0)))));

                // driverPad.b().whileTrue(drivetrain.pointAtHubComm5and(() ->
                // -driverPad.getLeftY() * MaxSpeed, () -> -driverPad.getLeftX() * MaxSpeed));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}