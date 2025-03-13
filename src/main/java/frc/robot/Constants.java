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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int ELEVATOR_MOTOR1_ID = 1;
    public static final int ELEVATOR_MOTOR2_ID = 2;
    public static final double elevatorPulleyDiameterInches = 2.0;
    public static final double elevatorGearRatio = 15.0;
    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = 60;
    public static final double inchesPerMotorRotation = elevatorPulleyDiameterInches * Math.PI/elevatorGearRatio;
    public static final double maxVelolicityRPM = 1800/inchesPerMotorRotation; // 1800 in/s (2.5 ft/s)= max linear velocity of elevator
    public static final double kMaxLinearAccelInchesPerSSquared = 0;
    public static final double kMaxLinearRateInchesPerS = 0;
    public static final double kElevatorKd = 0;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKp = 0;}
  
  public static class ArmConstants {
    public static final int ARM_MOTOR_ID = 3;
    public static final double ARM_GEAR_RATIO = 7.5; // Adjust as needed
    public static final double ARM_MAX_ANGLE = 165.0; // Maximum angle in degrees
    public static final double ARM_MIN_ANGLE = 0.0; // Minimum angle in degrees
    public static final double ARM_SPEED = 1.0; // Speed of the arm movement
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 4;
    public static final double CORAL_INTAKE_SPEED = 0.30;
    public static final double CORAL_OUTPUT_SPEED = 1.0 ; //speed of the coral output
    public static final double ALGAE_INTAKE_SPEED = 1.0; // Speed of the intake
    public static final double INTAKE_GEAR_RATIO = 1.0; // Adjust as needed
    public static final double INTAKE_THRESHOLD_DISTANCE = 3.0; // Maximum distance in inches
    public static final int CAN_RANGE_ID = 5;
    public static final double CORAL_INTAKE_TIME = 1.0; // Time in seconds
  }
}
