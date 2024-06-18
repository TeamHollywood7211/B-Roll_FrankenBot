package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
    public static final double stickDeadband = 0.1;;

    public static class IntakeConstants
    {
      public static final int IntakeMotor1ID = 41;
      public static final int IntakeMotor2ID = 42;
      public static final double speedReduction = 0.6;
      public static final int IntakeIR1ID = 0;
    } 
    public static class ShooterConstants
    {
      public static final int ShooterMotor1ID = 45;
      public static final int ShooterMotor2ID = 46;
    }
    public static class FeederConstants
    {
      public static final int Motor1ID = 41;
      public static final int Motor2ID = 42;
    }
    public static class ArmConstants
    {
      public static final int ArmMotor = 60;
      public static final double kP = 0.075;
      public static final double kI = 0;
      public static final double kD = 0.005;
    }
    public static class GlobalConstants
    {
      public static final int currentLimit = 40;
    }

//41 42 are intake
//46 is feeder??



    


}
