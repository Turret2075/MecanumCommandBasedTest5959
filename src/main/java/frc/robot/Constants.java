// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public class Mecanum {
    //Cosas Mecanum
    public static final int kFrontLeftID = 5;
    public static final int kFrontRightID = 4;
    public static final int kRearLeftID = 3;
    public static final int kRearRightID = 2;
    public static final double kMaxSpeedWheel = 3.2;
    public static final double kIntegratorRange = 1.0f;
    public static final double kContinuousInput = 180.0f;
    public static final double kErrorToleranceChasis = 2.0f;
    public static final double kDeadband = 0.02;
    public static final double kMaxVolts = 12.0;
    public static final double kStrafeSolver = 1.5;
  }

  public class PatherPID{
    public static final double kP_PatherTrans = 4.5;
    public static final double kI_PatherTrans = 0.0;
    public static final double kD_PatherTrans = 0.0;
    public static final double kP_PatherRot = 1.6;
    public static final double kI_PatherRot = 0.0;
    public static final double kD_PatherRot = 0.18;
  }

  public class Mecanismos{
    public static final int kIntakePort = 11;
  }
   public class Encoders{
    public static final int kEncoderFL_A = 0;
    public static final int kEncoderFL_B = 1;
    public static final int kEncoderFR_A = 4;
    public static final int kEncoderFR_B = 5;
    public static final int kEncoderRL_A = 8;
    public static final int kEncoderRL_B = 9;
    public static final int kEncoderRR_A = 6;
    public static final int kEncoderRR_B = 7;
   }
   public class PIDFFs{
    public static final double kP_wheel = 0.01;
    public static final double kI_wheel = 0.0;
    public static final double kD_wheel = 0.0;
    public static final double kS_wheel = 0.4;
    public static final double kV_wheel = 3.2;
    public static final double kA_wheel = 0.0;
    public static final double kP_chassis = 0.028;
    public static final double kI_chassis = 0.0;
    public static final double kD_chassis = 0.0032;
   }

   public class PuertoControl{
    public static final int kDriverControllerPort = 0;
   }
}
