package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
    public static final double stickDeadband = 0.1;; //Two semicolons just for safety 

    public static class IntakeConstants
    {
      public static final int IntakeMotor1ID = 41; 
      public static final int IntakeMotor2ID = 42; 
      public static final int ampMotorID = 43;

      public static final int IntakeIR1ID = 0; //IR that is possibly going to be unused

      public static final double speedReduction = 0.6; // This is my global slow down of the intake motors
    } 
    public static class ShooterConstants
    {
      public static final int ShooterMotor1ID = 45;
      public static final int ShooterMotor2ID = 46;
    }
    public static class ArmConstants
    {
      public static final int ArmMotor = 60;
      public static final double kP = 0.072; //PID for arm (possibly tune this better later)
      public static final double kI = 0; //I dont touch this value
      public static final double kD = 0.0137;
    }
    public static class GlobalConstants
    {
      public static final int currentLimit = 40; //A global current limit applied to all motors 
      public static final double joystickOverride = 4; // Overrides the joystick thingy 
    }

//41 42 are intake
//46 is feeder??



    


}
