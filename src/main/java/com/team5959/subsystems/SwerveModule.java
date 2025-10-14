package com.team5959.subsystems;

import com.team5959.SwerveModuleConstants;
import com.team5959.Constants.SwerveConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    //INITIALIZATION
    public int moduleID;

    //initialize motors
    private SparkMax driveMotor;
    private SparkMax rotationMotor;

    //initialize motor configurations
    private SparkBaseConfig driveConfig;
    private SparkBaseConfig rotationConfig; //SparkBaseConfig provides a general configuration framework for various motor controllers, while SparkMaxConfig extends this framework to include advanced features and settings specific to the SparkMax motor controller.

    //initialize encoders
    private CANcoder absoluteEncoder;
    private RelativeEncoder driveEncoder;

    //initialize PID Controller for rotation
    private PIDController rotationPID;

    //init info
    private double encOffset;

public SwerveModule (int moduleID, SwerveModuleConstants moduleConstants){
    this.moduleID = moduleID; //for easy differentation of modules
    encOffset = moduleConstants.angleOffset; //offset for correct aligment of the wheels when turned on.

    //instatiate drive motor, encoder and configuration
    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless); 
    driveEncoder = driveMotor.getEncoder();

    driveConfig = new SparkMaxConfig();
    driveConfig.inverted(moduleConstants.driveInverted);
    driveConfig.idleMode(IdleMode.kBrake); // driveMotor.setIdleMode(IdleMode.kBrake);
    driveConfig.smartCurrentLimit(25); //driveMotor.setSmartCurrentLimit(25); //set current limit to 25 amps to prevent browning out in the middle of driving 
   //set conversion factor for drive enc 
    driveConfig.encoder.velocityConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION); //reads velocity in meters per second instead of RPM
    driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION); //reads velocity in meters instead of rotations
   
    driveMotor.configure(driveConfig, null, null); //driveMotor.setInverted(moduleConstants.driveInverted);

   //instantiate rotation motor, absolute encoder and motor configuration
    rotationMotor = new SparkMax(moduleConstants.rotationMotorID, MotorType.kBrushless);
    absoluteEncoder = new CANcoder(moduleConstants.cancoderID);

    rotationConfig = new SparkMaxConfig();

    rotationConfig.inverted(moduleConstants.rotationInverted); //same as drive config
    rotationConfig.idleMode(IdleMode.kBrake);
    rotationConfig.smartCurrentLimit(25);

    rotationMotor.configure(rotationConfig, null, null); //        rotationMotor.setInverted(moduleConstants.rotationInverted);
   
    //configure rotation absolute encoder 
    absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5)); //abs enc is now +-180
    //absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(moduleConstants.angleOffset)); //implements encoder offset
    absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)); //positive rotation occurs when magnet is spun counter-clockwise when observer is facing the LED side of CANCoder

    //configure rotation PID controller 
    rotationPID = new PIDController(
        SwerveConstants.KP_TURNING, 
        SwerveConstants.KI_TURNING, 
        SwerveConstants.KD_TURNING);
    rotationPID.enableContinuousInput(-180, 180); //Continuous input considers min & max to be the same point; calculates the shortest route to the setpoint 

}
    //GET METHODS
    public double driveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double drivePosition() {
        return driveEncoder.getPosition();
    }   

    public double getAbsoluteEncoderDegrees() {
        return (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360) - encOffset;
    }

    //returns a new SwerveModuleState representing the current drive velocity and rotation motor angle 
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    //returns a new SwerveModulePosition representing the current drive position and rotation motor angle 
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivePosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    //SET METHODS

    public void setState(SwerveModuleState desiredState) {
        //optimize state so the rotation motor doesnt have to spin as much 
    
        SwerveModuleState optimizedState = desiredState;
        optimizedState.optimize(getState().angle);

        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        rotationMotor.set(rotationOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.MAX_SPEED * SwerveConstants.VOLTAGE); 

        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] DESIRED ANG DEG", getState().angle.getDegrees());
    }

    public void setAngle(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = desiredState;
        optimizedState.optimize(getState().angle);

        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        rotationMotor.set(rotationOutput); 
        driveMotor.set(0);
    }

    public void stop(){
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void print() {
    SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ABS ENC DEG", getAbsoluteEncoderDegrees());
    SmartDashboard.putNumber("S["+absoluteEncoder.getDeviceID()+"] DRIVE SPEED", driveVelocity());
    SmartDashboard.putNumber("S["+absoluteEncoder.getDeviceID()+"] ROTATION SPEED", absoluteEncoder.getVelocity().getValueAsDouble());
    SmartDashboard.putString("S["+absoluteEncoder.getDeviceID()+"] CURRENT STATE", getState().toString());
    }   
}
