// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;


public class FeederSubsystem extends SubsystemBase {



  CANSparkFlex Motor1 = new CANSparkFlex(FeederConstants.Motor1ID, MotorType.kBrushless);
  CANSparkFlex Motor2 = new CANSparkFlex(FeederConstants.Motor2ID, MotorType.kBrushless);



  public FeederSubsystem() {}

  

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
  public void setMotor(double speed)
  {
    Motor1.set(speed);
    Motor2.set(speed);
  }







}

