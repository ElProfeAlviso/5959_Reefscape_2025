package com.team5959.subsystems;

import com.team5959.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class IntakeSubsystem extends SubsystemBase {
    //INIZIALIZATION

    //initialize motors
    private final SparkMax coralIntakeMotorRight;
    private final SparkMax coralIntakeMotorLeft;

    private final SparkMax algaeIntakeMotor;

    //initialize motor configuration
    private final SparkBaseConfig coralIntakeMotorRightConfig;
    private final SparkBaseConfig coralIntakeMotorLeftConfig;
    private final SparkBaseConfig algaeIntakeMotorConfig;

    public IntakeSubsystem (){

        //instatiate motors and config
        coralIntakeMotorRight = new SparkMax(IntakeConstants.coralIntakeMotorRightID, MotorType.kBrushless);
        coralIntakeMotorLeft = new SparkMax(IntakeConstants.coralIntakeMotorLeftID, MotorType.kBrushless);

        algaeIntakeMotor = new SparkMax(IntakeConstants.algaeIntakeMotorID, MotorType.kBrushless);

        coralIntakeMotorRightConfig = new SparkMaxConfig();
        coralIntakeMotorLeftConfig = new SparkMaxConfig();
        algaeIntakeMotorConfig = new SparkMaxConfig();

        coralIntakeMotorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        coralIntakeMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        coralIntakeMotorLeftConfig.follow(coralIntakeMotorRight, IntakeConstants.coralIntakeMotorLeftInverted);
        coralIntakeMotorRightConfig.inverted(IntakeConstants.coralIntakeMotorRightInverted);
        algaeIntakeMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        coralIntakeMotorRight.configure(coralIntakeMotorRightConfig, null,null);
        coralIntakeMotorLeft.configure(coralIntakeMotorLeftConfig, null,null);
        algaeIntakeMotor.configure(algaeIntakeMotorConfig, null,null);
    }

    public void runCoralIntake(double speed){
        coralIntakeMotorRight.set(speed);
    }

    public void runAlgaeIntake(double speed){
        algaeIntakeMotor.set(speed);
    }

    public void stopCoralIntake(){
        coralIntakeMotorRight.set(0);
    }

    public void stopAlgaeIntake(){
        algaeIntakeMotor.set(0);
    }
}