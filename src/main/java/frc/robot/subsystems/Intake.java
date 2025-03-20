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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Intake extends SubsystemBase {

  //Initialize the intake motor
  private final TalonFX IntakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
  private final TalonFXConfiguration IntakeTalonFXConfig = new TalonFXConfiguration();

  //Initialize the CAN range
  private final CANrange intakeCANrange = new CANrange(IntakeConstants.CAN_RANGE_ID);
  
  
  //variables for coral detection
  private boolean hasCoral = false;
  private Timer intakeTimer = new Timer();




  public Intake() {

    IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    IntakeMotor.setInverted(true);
    IntakeMotor.getConfigurator().apply(IntakeTalonFXConfig);

    

  }

//Run once per scheduler run

  @Override
  public void periodic() {
     SmartDashboard.putBoolean("CANrange Status", hasCoral);
  }




  // Get the status signal of the CANRange sensor (True if object is within proximity range)
  //Set proximity in meters in TunerX for the CANRange sensor (.05 meters)

  public boolean DetectCoral(){
    return intakeCANrange.getIsDetected().getValue();
  }



  public void RunIntake(double speed) {

    hasCoral = DetectCoral();

    if(hasCoral && intakeTimer.get() < IntakeConstants.CORAL_INTAKE_STOP_TIME){
      intakeTimer.start();

      speed = MathUtil.clamp(speed, -1, 1);
      IntakeMotor.set(speed);
      
    }else if(hasCoral && intakeTimer.get() >= IntakeConstants.CORAL_INTAKE_STOP_TIME){ //Run the intake for a small amount of time after coral is detected
      stopIntake();
      intakeTimer.reset();

    }else{ 
      speed = MathUtil.clamp(speed, -1, 1);
      IntakeMotor.set(speed);
    }

  }

  public void stopIntake() {
    IntakeMotor.set(0);
  }

}
