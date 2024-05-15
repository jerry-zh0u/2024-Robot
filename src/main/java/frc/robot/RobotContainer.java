// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.io.BufferedWriter;
// import java.io.FileWriter;
// import java.io.IOException;
// import java.io.PrintWriter;
// import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

// import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoIntakeCommandOff;
import frc.robot.commands.AutoIntakeCommandOn;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.AutoShooterFirst;
import frc.robot.commands.CameraGoToTagCommand;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.IntakeCommandDown;
import frc.robot.commands.IntakeCommandMax;
import frc.robot.commands.IntakeCommandOff;
import frc.robot.commands.IntakeCommandOn;
import frc.robot.commands.IntakeCommandReverse;
import frc.robot.commands.IntakeCommandUp;
import frc.robot.commands.IntakeOffCommand;
import frc.robot.commands.IntakeZeroCommand;
import frc.robot.commands.ShooterAngleOff;
import frc.robot.commands.ShooterCommandOff;
import frc.robot.commands.ShooterCommandOn;
import frc.robot.commands.ShooterCommandReverse;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;
import frc.robot.commands.AutoShooterHelperOn;




public class RobotContainer {
 
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandPS4Controller joystick = new CommandPS4Controller(0); // My joystick
  private final CommandXboxController opController = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final SolenoidSubsystem solenoid = new SolenoidSubsystem();
  private final PhotonCamera m_Camera;
  private final SendableChooser<Command> auto = new SendableChooser<>();
  
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private double distance;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    // intake.setDefaultCommand(new ShooterCommandOff(intake));
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.circle().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rpotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.L1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    joystick.square().toggleOnTrue(new IntakeCommandOn(intake, solenoid));
    joystick.square().toggleOnFalse(new IntakeCommandOff(intake, solenoid));
    // joystick.triangle().toggleOnTrue(new IntakeZeroCommand(intake));
    // joystick.triangle().toggleOnTrue(new CameraGoToTagCommand(drivetrain, joystick.getLeftY()));
    // joystick.triangle().toggleOnTrue(new AutoDrive(drivetrain, drive, 1, 1));
    // joystick.triangle().onTrue(drivetrain.applyRequest(() -> drive.withVelocityX(2).withVelocityY(2).withRotationalRate(2)));
    joystick.R2().whileTrue(drivetrain.applyRequest(() -> CameraGoToTag()));
    //other intake button format
    // joystick.R1().toggleOnTrue(new PnumIntakeOn(inta))

    opController.leftTrigger().onTrue(new IntakeCommandReverse(intake));
    opController.leftTrigger().onFalse(new IntakeCommandOff(intake, solenoid));
    opController.rightTrigger().onTrue(new IntakeCommandMax(intake));
    opController.rightTrigger().onFalse(new IntakeCommandOff(intake, solenoid));
    opController.b().toggleOnTrue(new IntakeCommandOn(intake, solenoid));
    opController.b().toggleOnFalse(new IntakeCommandOff(intake, solenoid));
    opController.y().toggleOnTrue(new ShooterCommandOn(intake));
    opController.y().toggleOnFalse(new ShooterCommandOff(intake));
    opController.a().toggleOnTrue(new ShooterCommandReverse(intake));
    opController.a().toggleOnFalse(new ShooterCommandOff(intake));
    // opController.x().toggleOnTrue(new AutoShoot(intake));
    // opController.x().toggleOnFalse(new IntakeOffCommand(intake));
    opController.rightBumper().toggleOnTrue(new IntakeCommandDown(intake));
    opController.rightBumper().toggleOnFalse(new ShooterAngleOff(intake));
  opController.leftBumper().toggleOnTrue(new IntakeCommandUp(intake));
    opController.leftBumper().toggleOnFalse(new ShooterAngleOff(intake)); 
    
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    // auto.addOption("Taxi", new AutoDrive(drivetrain, drive, 1, 1));
    m_Camera = new PhotonCamera("April");
     // Register Named Commands
    //NamedCommands.registerCommand("AutoShooterCommandOff", new ShooterCommandOff(intake));
   NamedCommands.registerCommand("Intake", new SequentialCommandGroup(new AutoIntakeCommandOn(intake, solenoid), new WaitCommand(1), new IntakeCommandOff(intake, solenoid)));
    NamedCommands.registerCommand("Shooter", new SequentialCommandGroup(new ShooterCommandOn(intake), new WaitCommand(1.25), new AutoShooterHelperOn(), new WaitCommand(0.75), new ShooterCommandOff(intake))); //2 seconds actuation
   // NamedCommands.registerCommand("AutoShooterCommandOn", new ShooterCommandOn(intake));
    NamedCommands.registerCommand("AutoShooterFirst", new AutoShooterFirst(intake)); //2.7 second wait
    // NamedCommands.registerCommand("AutoIntakeCommandOff", new AutoIntakeCommandOff(intake, solenoid)); // 1 second wait
    // NamedCommands.registerCommand("AutoIntakeCommandOn", new AutoIntakeCommandOn(intake, solenoid));
    // NamedCommands.registerCommand("WaitCommand1", new WaitCommand(1));
    // NamedCommands.registerCommand("WaitCommand2", new WaitCommand(5));
    // NamedCommands.registerCommand("IntakeCOmmand", new AutoIntakeCommand(intake, solenoid));
    // Field2d field = new Field2d();
    // SmartDashboard.putData("field", field);

    // // Path
    // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
    //   field.setRobotPose(pose);
    // });

    // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
    //   field.getObject("target pose").setPose(pose);
    // });

    // PathPlannerLogging.setLogActivePathCallback((poses) -> {
    //   field.getObject("path").setPoses(poses);
    // });

    SmartDashboard.putData("Autochoices", auto);
    configureBindings();
  }

// private void configureAutoCommands(){
  //   Constants.DriveConstants.AUTO_EVENT_MAP.put("event 1", new PrintCommand("passed marker 1"));
    
  //   List<PathPlannerPath> auto1Paths = PathPlannerAuto.getPathGroupFromAutoFile("New Auto");
  //   Command autoTest = new SequentialCommandGroup(
  //     new FollowPathWith  
  //   )
  // }
  public Command getAutonomousCommand(){
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Drive Test");
    return new PathPlannerAuto("RS1");
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Blue Amp Auto 2p");
    // PathPlannerPath path = PathPlannerPath.fromPathFile("New New Auto");
    // System.err.println("-----------------------");
    // System.err.println(drivetrain.autoChooser.getSelected().getName());
    // return drivetrain.autoChooser.getAutoSelected();
    // return AutoBuilder.followPath(path);
    // return new PathPlannerAuto("SA1");
    //return auto.getSelected();
    // return drivetrain.applyRequest(() -> drive.withVelocityX(2));
    // return Commands.print("No autonomous command configured");
  }
  public SwerveRequest.RobotCentric CameraGoToTag() {
    var latestResult = m_Camera.getLatestResult();
    List<PhotonTrackedTarget> targets = latestResult.getTargets();
    PIDController pidControllerDrive = new PIDController(0.11, 0, 0.2);
    PIDController pidControllerAngle = new PIDController(0.11, 0, 0.00);
    System.err.println(targets.size());
    // Handle the case when no targets are detected
    if (targets.isEmpty()) {
      return drive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);
    }

    for (PhotonTrackedTarget target : targets) {
      int targetID = target.getFiducialId();
      // System.err.println("targetID" + " " + targetID);

      if (targetID == 4 || targetID == 7) {
        // System.err.println("=================");
        double targetYaw = target.getYaw();
      double centeringAdjust = -pidControllerAngle.calculate(targetYaw, 0);
        distance = PhotonUtils.calculateDistanceToTargetMeters(
          LimeLightConstants.CAMERA_HEIGHT_METERS, 
          LimeLightConstants.TARGET_HEIGHT_METERS, 
          LimeLightConstants.CAMERA_PITCH_RADIANS, 
          Units.degreesToRadians(target.getPitch()));
        // SmartDashboard.putNumber("Distance", distance);
        // System.out.println(distance);
        double forwardSpeed = pidControllerDrive.calculate(distance, LimeLightConstants.GOAL_RANGE_METERS);
        // System.out.println(forwardSpeed);
        return drive.withVelocityX(forwardSpeed)
              .withVelocityY(/*-joystick.getLeftX() * MaxSpeed*/ centeringAdjust )
              .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
      }
    }

    // Default behavior when no matching targets are found
    return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
          .withVelocityY(-joystick.getLeftX() * MaxSpeed)
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
  }
}
