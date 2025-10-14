package com.team5959.subsystems;

import com.team5959.Constants;
import com.team5959.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;

//Sensores CAN CTRE
import com.ctre.phoenix6.CANBus; //Sensor de rango y proximidad CANrange
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class ElevatorSubsytem extends SubsystemBase{
    //INITIALIZATION

     // Creacion de objeto de sensor de distancia y deteccion de objetos CANrange
  private final CANBus kCANBus = new CANBus("rio");
  private final CANrange canRange = new CANrange(10, kCANBus);

    //initialize motors
    private final SparkMax elevatorRight;
    private final SparkMax elevatorLeft;

    //initialize motor configuration
    private final SparkBaseConfig elevatorLeftConfig;
    private final SparkBaseConfig elevatorRightConfig;

    //initialize encoder
    private final RelativeEncoder elevatorEncoder;

    //initialize PID controller
    private final PIDController elevatorPID;
    private final PIDController elevatorStartingPositionPID;

    //Target position
    private double targetPosition;
    private boolean isManualMode = false;

    //LIMITS
    //Soft Limit Configuration
    SoftLimitConfig elevatorRightSoftLimitConfig;
    SoftLimitConfig elevatorLeftSoftLimitConfig;

    //Digital sensors
    DigitalInput digitalUpperLimitSwitch;
    DigitalInput digitalDownLimitSwitch;
    boolean elevatorUpperLimitSwitch;
    boolean elevatorDownLimitSwitch;

    StatusSignal<Boolean> coralIsDetected = canRange.getIsDetected(false);

    public ElevatorSubsytem(){

         // Configuracion de sensor CanRange
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of at least 2000 its valid.
    config.ProximityParams.ProximityThreshold = 0.1; // If CANrange detects an object within 0.2 meters, it will trigger
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the CANrange update as fast as possible at
    canRange.getConfigurator().apply(config);// Apply the configuration to the CANrange

   


        //instatiate motors, config and encoder
        elevatorRight = new SparkMax(ElevatorConstants.elevatorRightID, MotorType.kBrushless);
        elevatorLeft = new SparkMax(ElevatorConstants.elevatorLeftID, MotorType.kBrushless);

        elevatorEncoder = elevatorRight.getEncoder();

        elevatorEncoder.setPosition(0);
    
        elevatorPID = new PIDController(Constants.ElevatorConstants.KP_ELEVATOR, Constants.ElevatorConstants.KI_ELEVATOR, Constants.ElevatorConstants.KD_ELEVATOR);
        elevatorStartingPositionPID = new PIDController(ElevatorConstants.KP_SP_ELEVATOR, ElevatorConstants.KI_ELEVATOR, ElevatorConstants.KP_ELEVATOR);
        
        elevatorRightConfig = new SparkMaxConfig();
        elevatorLeftConfig = new SparkMaxConfig();
        elevatorRightSoftLimitConfig = new SoftLimitConfig();
        elevatorLeftSoftLimitConfig = new SoftLimitConfig();

        elevatorRightSoftLimitConfig.forwardSoftLimit(ElevatorConstants.elevatorLowerLimit); //positive values go down
        elevatorRightSoftLimitConfig.reverseSoftLimit(ElevatorConstants.elevatorUpperLimit); //negative values go up
        elevatorRightSoftLimitConfig.forwardSoftLimitEnabled(ElevatorConstants.forwardSoftLimitEnabled);
        elevatorRightSoftLimitConfig.reverseSoftLimitEnabled(ElevatorConstants.reverseSoftLimitEnabled);

        elevatorRightConfig.apply(elevatorRightSoftLimitConfig);

        elevatorLeftConfig.follow(elevatorRight, ElevatorConstants.elevatorLeftInverted);
        elevatorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        elevatorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        elevatorRightConfig.inverted(ElevatorConstants.elevatorRightInverted);

        elevatorLeft.configure(elevatorLeftConfig, null, null);
        elevatorRight.configure(elevatorRightConfig, null, null);        

        digitalUpperLimitSwitch = new DigitalInput(ElevatorConstants.elevatorUpperLimitSwitch);
        digitalDownLimitSwitch = new DigitalInput(ElevatorConstants.elevatorLowerLimitSwitch);
    }

    public void holdCurrentPosition() {
        // Set target to current position
        targetPosition = elevatorEncoder.getPosition(); 
    }

    //PRESET POSITIONS
    public void moveToStartingPosition() {
        // Move to preset position 0
        isManualMode = false;
        setTargetPosition(Constants.ElevatorConstants.elevatorStartingPosition);  
    }

    public void moveToL1Position() {     
        // Move to preset L1 position
        isManualMode = false;
        setTargetPosition(Constants.ElevatorConstants.elevatorL1Position);  
    }

    public void moveToL2Position() {
        // Move to preset L2 position
        isManualMode = false;
        setTargetPosition(Constants.ElevatorConstants.elevatorL2Position);  
    }

    public void moveToL3Position() {
        // Move to preset L3 position
        isManualMode = false;
        setTargetPosition(Constants.ElevatorConstants.elevatorL3Position);  
    }

    // Method to set a target position
    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void elevatorManualMode(double speed){
        isManualMode = true;
        if(elevatorUpperLimitSwitch){
           speed = MathUtil.clamp(speed,0,0.4);
        }else if(elevatorDownLimitSwitch){
            speed =  MathUtil.clamp(speed,-0.4,0);
        }   
        elevatorRight.set(speed);
    }

    public void stopElevator(){
        isManualMode = true;
        elevatorRight.set(0);
    }

  
    @Override
    public void periodic() {
        elevatorUpperLimitSwitch = !digitalUpperLimitSwitch.get();
        elevatorDownLimitSwitch = !digitalDownLimitSwitch.get();
        

        SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
        SmartDashboard.putBoolean("Sensor arriba", elevatorUpperLimitSwitch);
        SmartDashboard.putBoolean("Sensor abajo", elevatorDownLimitSwitch);
        SmartDashboard.putBoolean("CanRange Coral", coralIsDetected.getValue());

        // PID control mode
        if (!isManualMode) {
            if (targetPosition == -10) {
                double pidOutput = elevatorStartingPositionPID.calculate(elevatorEncoder.getPosition(), targetPosition);
                elevatorRight.set(pidOutput);
            } else {
                double pidOutput = elevatorPID.calculate(elevatorEncoder.getPosition(), targetPosition);
                elevatorRight.set(pidOutput);
            }
        } else {
            isManualMode = true;
        }
    // Set the motor to the calculated PID output
    }

    // Method to check if the motor has reached the target position
    public boolean atTargetPosition() {
        return elevatorPID.atSetpoint();
    }
}

