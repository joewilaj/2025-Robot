// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.OperatorConstants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Elevator extends SubsystemBase {

  private final SparkMax leaderMotor;
  private SparkMax followerMotor;
  private final SparkMaxConfig leaderConfig;
  private final SparkMaxConfig followerConfig;
  private final SparkClosedLoopController pidController;

  

  
  public Elevator(){

    leaderMotor = new SparkMax(OperatorConstants.ELEVATOR_MOTOR1_ID, MotorType.kBrushless);
    followerMotor = new SparkMax(OperatorConstants.ELEVATOR_MOTOR2_ID, MotorType.kBrushless);

    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();

    pidController = leaderMotor.getClosedLoopController();

    leaderConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .encoder.positionConversionFactor(OperatorConstants.inchesPerMotorRotation)
        .velocityConversionFactor(OperatorConstants.inchesPerMotorRotation);


    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(OperatorConstants.kElevatorKp)
        .i(OperatorConstants.kElevatorKi)
        .d(OperatorConstants.kElevatorKd)
        .outputRange(-1, 1)
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5500, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    
  
    followerConfig
        .follow(leaderMotor,true);

    
    
    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);



  }

 public double getHeightInches() {
	return leaderMotor.getEncoder().getPosition();
  }


  public void setElevatorSpeed(double speed){
    // Get current position to check if we're at limits
    double currentPosition = getHeightInches();
    
    // Prevent motion beyond limits
    if ((currentPosition >= OperatorConstants.MAX_HEIGHT && speed > 0) || 
        (currentPosition <= OperatorConstants.MIN_HEIGHT && speed < 0)) {
      // Stop at limits
      pidController.setReference(0, SparkMax.ControlType.kVelocity);
    } else {
      // Otherwise apply requested velocity
      pidController.setReference(OperatorConstants.maxVelolicityRPM*speed, SparkMax.ControlType.kVelocity);
    };
  }

  public void stopElevator(){
    leaderMotor.set(0);
  }


  public void setElevatorPosition(double position){
    double safeposition = MathUtil.clamp(position, OperatorConstants.MIN_HEIGHT, OperatorConstants.MAX_HEIGHT);
    pidController.setReference(safeposition,ControlType.kPosition);
  }

  
  public void resetElevatorPosition() {
	leaderMotor.getEncoder().setPosition(0);
  }

  public boolean isAtHeight(double targetHeight, double tolerance) {
	double currentHeight = getHeightInches();
	return Math.abs(currentHeight - targetHeight) <= tolerance;
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    

  
  

  }
  


}
