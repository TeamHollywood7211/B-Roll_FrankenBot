// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeShooterCommand;
import frc.robot.generated.TunerConstants;
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


  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Commands
  private final IntakeShooterCommand m_IntakeShooterCommand = new IntakeShooterCommand(m_shooter, m_intake, m_driver);


  // AUTO COMMANDS

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

    drivetrain.setDefaultCommand(
        // #region standard drivetrain
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    m_driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_driver.getLeftY(), -m_driver.getLeftX()))));

    //Resets gyro
    m_driver.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // drivetrain.registerTelemetry(logger::telemeterize);

    // The slow mode type things we implemented
    m_driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    m_driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    m_driver.pov(270).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));
    m_driver.pov(90).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));

    /*m_driver.rightBumper().whileTrue( // Auto target for driver
        drivetrain.applyRequest(() -> autoAim.withTargetDirection(drivetrain.directionToGoal())
            .withVelocityX(-m_driver.getLeftY() * MaxSpeed)
            .withVelocityY(-m_driver.getLeftX() * MaxSpeed)

        )
    );*/

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
    m_driver.start().and(m_driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driver.start().and(m_driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));



    new Trigger(m_driver.leftBumper()).onTrue(m_IntakeShooterCommand);
    new Trigger(m_driver.leftTrigger()).onTrue(m_IntakeShooterCommand);
    new Trigger(m_driver.rightBumper()).onTrue(m_IntakeShooterCommand);
    new Trigger(m_driver.rightTrigger()).onTrue(m_IntakeShooterCommand);
    


    // new Trigger(m_operator.x()).onTrue(new
    // InstantCommand(limelightSubsystem::findDistanceToTarget));

    // new Trigger(m_operator.x()).onTrue(new
    // InstantCommand(climberSubsystem::raiseClimbers));
    // new Trigger(m_operator.a()).onTrue(new
    // InstantCommand(climberSubsystem::lowerClimbers));

    // manual 3rd controller

    // new Trigger(m_dev.a()).onTrue(m_climber);
    // new Trigger(m_dev.y()).whileTrue(m_climber);
    // new Trigger(m_dev.b()).whileTrue(new
    // InstantCommand(climberSubsystem::resetClimberZero));


    //button box stuff
/* *
    new Trigger(m_buttonLeft.button(5)).onTrue(new InstantCommand(armSubsystem::posZero));
    new Trigger(m_buttonLeft.button(4)).onTrue(new InstantCommand(armSubsystem::posMid));
    new Trigger(m_buttonLeft.button(3)).onTrue(new InstantCommand(armSubsystem::posAmp));
    new Trigger(m_buttonLeft.button(2)).onTrue(new InstantCommand(armSubsystem::posClimb));

    
    new Trigger(m_buttonLeft.button(9)).onTrue(new InstantCommand(armSubsystem::posLong));
    new Trigger(m_buttonLeft.button(7)).onTrue(new InstantCommand(armSubsystem::posExLong));
    new Trigger(m_buttonLeft.button(8)).onTrue(new InstantCommand(armSubsystem::posHPS));
    new Trigger(m_buttonLeft.button(6)).onTrue(new InstantCommand(armSubsystem::posCross));

  
    //manual stuff
    new Trigger(m_buttonRight.button(1)).onTrue(new InstantCommand(shooterSubsystem::auto_shooterOff));
    new Trigger(m_buttonRight.button(1)).onTrue(new InstantCommand(intakeSubsystem::auto_intakeOff));

    //new Trigger(m_buttonLeft.button(6)).onTrue(new InstantCommand(armSubsystem::pos));

    //Shooter/Intake

    new Trigger(m_buttonRight.button(7)).onTrue(m_intakeShooterCommand);
    new Trigger(m_buttonRight.button(8)).onTrue(m_intakeShooterCommand);
    new Trigger(m_buttonRight.button(9)).onTrue(m_intakeShooterCommand);
    new Trigger(m_buttonRight.button(10)).onTrue(m_intakeShooterCommand); */

  }

  public RobotContainer() {

    //// Auto Commands////
    // Auto Actions




    // Work on auto targetting to add the following commands
    /*
     * aim_speaker = aiming at speaker
     * 
     */
    // NamedCommands.registerCommand("aim_speaker", new
    // InstantCommand(drivetrain::aimSpeaker));

    // turns out you have to put autochooser AFTER the named commands thingy
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    createFrontUsbCamera();
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
