// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class IntakeShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;
  private final CommandXboxController m_controller;
  private final IntakeSubsystem m_intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeShooterCommand(ShooterSubsystem shooter, IntakeSubsystem intake,CommandXboxController controller) {
    m_shooter = shooter;
    m_controller = controller;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(boolToInt(m_controller.leftBumper().getAsBoolean()) - boolToInt(m_controller.leftTrigger().getAsBoolean()));
    /*\
    |*| -Gets the difference between the bumper and the trigger 
    |*|     /\  /\
    |*|    /  \/  \
    |*|   <  o  o  >
    |*|    \  w   /
    |*|     ######
    \*/
    
    m_shooter.setMotor(boolToInt(m_controller.leftBumper().getAsBoolean()) - boolToInt(m_controller.leftTrigger().getAsBoolean()));
    m_intake.setMotor(boolToInt(m_controller.rightBumper().getAsBoolean()) - boolToInt(m_controller.rightTrigger().getAsBoolean()));
    
    



  }

  public int boolToInt(boolean b) //theres probably a way to do this in Java but theres just some dumb moments where some values wont take bools as ints
  {
    if(b == false)
    {
      return 1;
    }
    else{
      return 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
