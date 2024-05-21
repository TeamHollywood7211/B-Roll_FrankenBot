// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {



  CANSparkFlex IntakeMotor1 = new CANSparkFlex(IntakeConstants.IntakeMotor1ID, MotorType.kBrushless);
  CANSparkFlex IntakeMotor2 = new CANSparkFlex(IntakeConstants.IntakeMotor2ID, MotorType.kBrushless);



  public IntakeSubsystem() {
    IntakeMotor1.setSmartCurrentLimit(40);
    IntakeMotor2.setSmartCurrentLimit(40);
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
    IntakeMotor1.set(speed);
    IntakeMotor2.set(-speed);
  }







}

