// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.GlobalConstants;
import frc.robot.commands.IntakeShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class RobotContainer {
  private static final double MaxSpeed = 1;
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  static Rotation2d cachedAngle = Rotation2d.fromDegrees(0);
  public static double shooterSpeed = 0.65;
  private final SendableChooser<Command> autoChooser;

  // Subsystems


  private final CommandXboxController m_driver = new CommandXboxController(0); //Driver joystick
  private final CommandXboxController m_operator = new CommandXboxController(1); //Operator joystick

  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Commands
  private final IntakeShooterCommand m_IntakeShooterCommand = new IntakeShooterCommand(m_shooter, m_intake, m_driver);


  /////////  
  // AUTO COMMANDS
  /////////


  // Auto Aim Shenanigans
  private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle();
  private final PhoenixPIDController autoTurnPID = new PhoenixPIDController(3.2, 0, 0.2);

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() //Creates the freaking swerve dude!!!

      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public void createFrontUsbCamera() {
    CameraServer.startAutomaticCapture(); //Camera stuff :3
  }

  /* Path follower */
  // private Command runAuto = drivetrain.getAutoPath("Tests");

  // private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    autoAim.HeadingController = autoTurnPID;
    autoAim.HeadingController.enableContinuousInput(-180, 180);

    System.out.println(m_driver.getLeftY());

    drivetrain.setDefaultCommand(
        // #region standard drivetrain
        // Drivetrain will execute this command periodically\
        
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driver.getLeftY() * GlobalConstants.joystickOverride) // Drive forward with
                                                                                           // negative Y (forward)
            
            .withVelocityY(-m_driver.getLeftX() * GlobalConstants.joystickOverride) // Drive left with negative X (left)
            .withRotationalRate(-m_driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    //m_driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_driver.getLeftY(), -m_driver.getLeftX()))));

    //Resets gyro
    m_driver.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // drivetrain. registerTelemetry(logger::telemeterize);

    // The slow mode type things we implemented
    // m_driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // m_driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    // m_driver.pov(270).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));
    // m_driver.pov(90).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));

    

    // m_driver.rightBumper().whileTrue(drivetrain.run(() ->
    // drivetrain.drive_autoAim(0)));

    /* Bindings for drivetrain characterization */
    /*
     * These bindings require multiple buttons pushed to swap between quastatic and
     * dynamic
     */
    /*
     * Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction
     */
    m_driver.back().and(m_driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driver.back().and(m_driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //amongus
    m_driver.start().and(m_driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driver.start().and(m_driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    

    new Trigger(m_driver.leftBumper()).onTrue(m_IntakeShooterCommand);
    new Trigger(m_driver.leftTrigger()).onTrue(m_IntakeShooterCommand);
    new Trigger(m_driver.rightBumper()).onTrue(m_IntakeShooterCommand);
    new Trigger(m_driver.rightTrigger()).onTrue(m_IntakeShooterCommand);
    


    new Trigger(m_driver.x()).onTrue(new InstantCommand(m_arm::toggleArm));

    new Trigger(m_driver.y()).onTrue(new InstantCommand(m_arm::armUp));
    new Trigger(m_driver.a()).onTrue(new InstantCommand(m_arm::armDown));

  }

  public RobotContainer() {

    ////AUTO ACTIONS FOR AUTON ////
    //NamedCommands.registerCommand("aim_speaker", new InstantCommand(drivetrain::aimSpeaker));
    NamedCommands.registerCommand("au_intake", new InstantCommand(m_intake::intakeStart));
    NamedCommands.registerCommand("au_intakestop", new InstantCommand(m_intake::intakeStop));
    NamedCommands.registerCommand("au_shoot", new InstantCommand(m_shooter::shooterStart));
    NamedCommands.registerCommand("au_shootstop", new InstantCommand(m_shooter::shooterStop));
    // turns out you have to put autochooser AFTER the named commands thingy
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //createFrontUsbCamera();
    configureBindings();
    //RobotController.setBrownoutVoltage(5);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
    // return runAuto;
  }
  public int boolToInt(double input)
  {
    if(input == 0)
    {
      return 1;
    }
    else{
      return 0;
    }
  }
}
