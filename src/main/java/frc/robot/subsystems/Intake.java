// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;




public class Intake extends SubsystemBase {
  /** Creates a new RotatingArm. */
  // Initialize the arm motor controller
  private final TalonFX IntakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
  private final TalonFXConfiguration IntakeTalonFXConfig = new TalonFXConfiguration();

  //Initialize the CAN range
  private final CANrange intakeCANrange = new CANrange(IntakeConstants.CAN_RANGE_ID);
  
  
  private boolean hasCoral = false;
  private boolean isIntaking = false;
  private Timer intakeTimer = new Timer();




  public Intake() {

    IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    //intakeMotor.setInverted(true);
    IntakeMotor.getConfigurator().apply(IntakeTalonFXConfig);

    

  }

//Run once per scheduler run

  @Override
  public void periodic() {

    updateCoralDetection(); 
  

    //Check if the intake has coral
   
    if (isIntaking && hasCoral) {
      stopIntake();
      isIntaking = false;
    }
  }

  public double getCANrangeDistance(){
    return intakeCANrange.getDistance().getValueAsDouble()*0.0254; // distance to nearest object converted to inches from meters
  }




  public boolean DetectCoral(){
    double currentDistanceInches = intakeCANrange.getDistance().getValueAsDouble()*0.0254; 
    boolean validReading = intakeCANrange.getFaultField().getValue() == 0;// distance to nearest object converted to inches from meters
  
    if(validReading && currentDistanceInches < IntakeConstants.INTAKE_THRESHOLD_DISTANCE){ 
      return true; 
    }
    else{
      return false; //If the nearest distance is greater than threshold, the CANRange is detecting the empty funnel
    }

  }

  public boolean hasCoral() {
    hasCoral = DetectCoral();
    return hasCoral;
  }


  public void RunIntake(double speed) {
    // clamp the speed to the allowed range
    speed = MathUtil.clamp(speed, -1, 1);
    IntakeMotor.set(speed);
    isIntaking = true;
    intakeTimer.start();
  }

  public void stopIntake() {
    IntakeMotor.set(0);
  }

  private void updateCoralDetection() {

    hasCoral = DetectCoral();
  }

  //method to run the intake at low coral intake speed for set amount of time
  public void IntakeWithTimeout(double timeout){
    if (!hasCoral) {
      RunIntake(IntakeConstants.CORAL_INTAKE_SPEED);
      
      if (intakeTimer.get() > timeout) {
        stopIntake();
      }
    } else {
      stopIntake();
      }
  }


}
