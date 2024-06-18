// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  
  //45 is a neo vortex
  //New motor and make it the same as intake
  

  CANSparkFlex motor1 = new CANSparkFlex(ShooterConstants.ShooterMotor1ID, MotorType.kBrushless);
  CANSparkMax  motor2 = new CANSparkMax (ShooterConstants.ShooterMotor2ID, MotorType.kBrushless);



  public ShooterSubsystem() {
    //motor1.setSmartCurrentLimit(GlobalConstants.currentLimit);
    motor2.setSmartCurrentLimit(GlobalConstants.currentLimit);
  }

  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
  public void setMotor(double speed)
  {
    //speed = speed; //Ryans epic idea
    motor1.set(speed);
    motor2.set(-speed);
  }

  public void shooterStart()
  {
    motor1.set(1);
    motor2.set(-1);
  }

  public void shooterStop()
  {
    motor1.set(0);
    motor2.set(0);
  }






}

