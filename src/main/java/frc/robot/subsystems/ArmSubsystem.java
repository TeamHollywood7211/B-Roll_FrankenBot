// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GlobalConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  PIDController armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

  CANSparkMax ArmMotor = new CANSparkMax(ArmConstants.ArmMotor, MotorType.kBrushless);
  
  public RelativeEncoder ArmEncoder = ArmMotor.getEncoder();

  double ArmSetpoint = ArmEncoder.getPosition();

  boolean armDown = true;

  
  public ArmSubsystem() {
    ArmMotor.setSmartCurrentLimit(GlobalConstants.currentLimit);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Pos", ArmEncoder.getPosition());
    ArmMotor.set(MathUtil.clamp(armPID.calculate(ArmEncoder.getPosition(), ArmSetpoint), -1, 1));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void toggleArm()
  {
    if(armDown)
    {
      ArmSetpoint = 124;  
    }
    else
    {
      ArmSetpoint = 0;
    }
    armDown = !armDown;
  }


  public void moveArm(int amount)
  {
    ArmSetpoint += amount;
  }

    public void armUp()
  {
    moveArm(5);
  }
    public void armDown()
  {
    moveArm(-5);
  }

  public void armZero()
  {
    //l maoooo I didnt finish this
  }


}
