// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {



  CANSparkFlex IntakeMotor1 = new CANSparkFlex(IntakeConstants.IntakeMotor1ID, MotorType.kBrushless);
  CANSparkFlex IntakeMotor2 = new CANSparkFlex(IntakeConstants.IntakeMotor2ID, MotorType.kBrushless);
  CANSparkMax AmpMotor = new CANSparkMax(IntakeConstants.ampMotorID, MotorType.kBrushed);

  //DigitalInput intakeIR = new DigitalInput(IntakeConstants.IntakeIR1ID);



  public IntakeSubsystem() {
    IntakeMotor1.setSmartCurrentLimit(GlobalConstants.currentLimit); //SETTING THE CURRENT LIMITS
    IntakeMotor2.setSmartCurrentLimit(GlobalConstants.currentLimit);
    AmpMotor.    setSmartCurrentLimit(GlobalConstants.currentLimit);
  }                

  

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
  public void setMotor(double speed)
  {
    speed = speed * IntakeConstants.speedReduction;
    IntakeMotor1.set(speed);
    IntakeMotor2.set(-speed);
    AmpMotor.set(speed);
  }
  public void intakeStart()
  {
    double speed = 1 * IntakeConstants.speedReduction;
    IntakeMotor1.set(speed);
    IntakeMotor2.set(-speed);
    AmpMotor.set(speed);
  }
   public void intakeStop()
  {
    IntakeMotor1.set(0);
    IntakeMotor2.set(0);
    AmpMotor.set(0);
  }


}

