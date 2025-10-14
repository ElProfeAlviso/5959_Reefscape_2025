// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TITANIUM RAMS 5959, FRC 2025
// Authors: 5959 Programming Team (Beatriz Marún, Jorge Pineda, Danna Hernández, Denis Cerón) & Mentor Sebastian León

package com.team5959;


import com.team5959.Constants.ControllerConstants;
import com.team5959.subsystems.SwerveChassis;
import com.team5959.subsystems.ElevatorSubsytem;
import com.team5959.subsystems.IntakeSubsystem;
import com.team5959.subsystems.LEDSubsystem;
import com.team5959.subsystems.ArmIntakeSubsystem;
import com.team5959.subsystems.MiniArmSubsystem;
import com.team5959.commands.SwerveDrive;
import com.team5959.commands.ArmIntakeCommand;
import com.team5959.commands.IntakeCommand;
import com.team5959.commands.ElevatorCommand;
import com.team5959.commands.MiniArmCommand;
import com.team5959.commands.SetLEDColorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


public class RobotContainer {

    
  //subsystems
  private final SwerveChassis swerveChassis = new SwerveChassis();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  private final ElevatorSubsytem elevatorSubsytem = new ElevatorSubsytem();
  private final MiniArmSubsystem miniArmSubsystem = new MiniArmSubsystem();

  //controllers
  private final PS4Controller control = new PS4Controller(ControllerConstants.kDriverControllerPort);
  private final GenericHID controlOp = new GenericHID(ControllerConstants.kOperatorControllerPort);

  //drive buttons
  private final JoystickButton resetNavxButton = new JoystickButton(control, 10);

  //Autonomous
  public static final String kForward = "Forward";
  public static final String kRight = "Right";
  public static final String kLeft = "Left";
  public static final String kBlueDiverStation1 = "Blue Drive Station 1";
  public static final String kBlueDriverStation2 = "Blue Drive Station 2";
  public static final String kBlueDriverStation3 = "Blue Drive Station 3";
  public static final String kRedDriverStation1 = "Red Drive Station 1";
  public static final String kRedDriverStation2 = "Red Drive Station 2";
  public static final String kRedDriverStation3 = "Red Drive Station 3";
  public static final String kTestDriverStation3 = "Test Driver Station 3";
  
  private final LEDSubsystem ledcitos = new LEDSubsystem(0); 
  //AXIS

  //Pathplanner
  public final SendableChooser<String> autoChooser;
  private PathPlannerPath path;
  public String autoChoose;

  public RobotContainer() {



    swerveChassis.setDefaultCommand(new SwerveDrive(swerveChassis, () -> -control.getLeftY(), () -> -control.getLeftX(), () -> control.getRightX(), true, ()-> control.getR1ButtonPressed()));
    intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem, elevatorSubsytem, ()-> controlOp.getRawAxis(2), ()-> controlOp.getRawAxis(3), ()-> control.getL2Axis(), ()-> control.getR2Axis()));
    elevatorSubsytem.setDefaultCommand(new ElevatorCommand(elevatorSubsytem, ()-> controlOp.getRawButton(1), ()-> controlOp.getRawButton(3), ()-> controlOp.getRawButton(4), ()-> controlOp.getRawButton(2),()-> controlOp.getRawButton(5), ()-> controlOp.getRawButton(6)));
    armIntakeSubsystem.setDefaultCommand(new ArmIntakeCommand(armIntakeSubsystem, ()-> control.getSquareButtonPressed(), ()-> control.getCrossButtonPressed()));
    miniArmSubsystem.setDefaultCommand(new MiniArmCommand(miniArmSubsystem, ()-> control.getTriangleButtonPressed(), ()-> control.getCircleButtonPressed()));
    
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Forward", kForward);
    autoChooser.addOption("Right", kRight);
    autoChooser.addOption("Left", kLeft);
    autoChooser.addOption("Blue Drive Station 1", kBlueDiverStation1);
    autoChooser.addOption("Blue Drive Station 2", kBlueDriverStation2);
    autoChooser.addOption("Blue Drive Station 3", kBlueDriverStation3);
    autoChooser.addOption("Red Drive Station 1", kRedDriverStation1);
    autoChooser.addOption("Red Drive Station 2", kRedDriverStation2);
    autoChooser.addOption("Red Drive Station 3", kRedDriverStation3);
    autoChooser.addOption("Test Driver Station 3", kTestDriverStation3);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
  }

  private void configureBindings() {
    resetNavxButton.onTrue(new InstantCommand(() -> swerveChassis.resetNavx()));

    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Forward"));

    
  }
  
  public void periodic(){
    
  }
  
  public Command getAutonomousCommand() {
    autoChoose = autoChooser.getSelected();
  try{
     

    switch (autoChoose) {
     case kForward:
       path = PathPlannerPath.fromPathFile("Forward");
       break;
     
     case kLeft:
       path = PathPlannerPath.fromPathFile("Left");
       break;

     case kRight:
       path = PathPlannerPath.fromPathFile("Right");
       break;

     case kBlueDiverStation1:
       path = PathPlannerPath.fromPathFile("Blue Driver Station 1");
       break;

     case kBlueDriverStation2:
       path = PathPlannerPath.fromPathFile("Blue Driver Station 2");
       break;

     case kBlueDriverStation3:
       path = PathPlannerPath.fromPathFile("Blue Driver Station 3");
       break;

     case kRedDriverStation1:
       path = PathPlannerPath.fromPathFile("Red Driver Station 1");
       break;

     case kRedDriverStation2:
       path = PathPlannerPath.fromPathFile("Red Driver Station 2");
       break;

     case kRedDriverStation3:
       path = PathPlannerPath.fromPathFile("Red Driver Station 3");
       break;

      case kTestDriverStation3:
        path = PathPlannerPath.fromPathFile("Test Driver Station 3");
        break;
      
     default:
       break;
    }

    return AutoBuilder.followPath(path);
  } catch (Exception e) {
    DriverStation.reportError("Error loading path: " + e.getMessage(), e.getStackTrace());
    return Commands.none();
  }
  }
}

