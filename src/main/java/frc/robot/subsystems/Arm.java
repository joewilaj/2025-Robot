// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;



public class Arm extends SubsystemBase {
  /** Creates a new RotatingArm. */
  // Initialize the arm motor controller
  private final TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
  private TalonFXConfiguration armTalonFXConfig = new TalonFXConfiguration()
    .withMotionMagic(new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(1)
      .withMotionMagicAcceleration(1));
  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(ArmConstants.ARM_ENCODER_PORT,360,0);
  private final PIDController pidController = new PIDController(ArmConstants.kp,ArmConstants.ki ,ArmConstants.kd);


  public Arm() {
    // Configure the arm motor to use motion magic control with FOC Postion
    armMotor.setNeutralMode(NeutralModeValue.Brake);
    armMotor.getConfigurator().apply(armTalonFXConfig);
    pidController.setTolerance(2.0);


    //code to set the amount of roatations based on the gear ratio
    

    
  }

  /* 
  //Set the target angle of the arm using motion magic
  public void setTargetAngle(double AngleDegrees) {

    // Clamp the Angle to the allowed range
        double SafeAngleDegrees = MathUtil.clamp(
        AngleDegrees,
        ArmConstants.ARM_MIN_ANGLE,
        ArmConstants.ARM_MAX_ANGLE);

    // Convert the angle from degrees to rotations
        double targetAngle_rotations = (SafeAngleDegrees/360)*ArmConstants.ARM_GEAR_RATIO;

        armMotor.setControl(new MotionMagicDutyCycle(targetAngle_rotations));
  }

*/


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Arm Angle", getAngle());

  }

  public void setArmSpeed(double speed) {

    armMotor.set(speed);
  }

  public void stopArm() {
    // Replace with actual logic to stop the arm
    armMotor.set(0);
  }


  //Get the current angle of the arm using the DutyCycleEncoder (Option 2)
  public double getAngle(){
    return armEncoder.get();

  }



}
