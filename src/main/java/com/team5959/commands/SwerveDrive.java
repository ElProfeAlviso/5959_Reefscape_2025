package com.team5959.commands;

import com.team5959.Constants.SwerveConstants;
import com.team5959.subsystems.SwerveChassis;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrive extends Command { //extends is used to indicate that a class is inheriting from a superclass. inherates properterties and behaviors
    private SwerveChassis swerveChassis;

    private DoubleSupplier xSupplier, ySupplier, zSupplier;
    private BooleanSupplier lockButtonSupplier;
    private boolean fieldOriented;

    //Chassis constructor
    public SwerveDrive(SwerveChassis swerveChassis, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier, boolean fieldOriented, BooleanSupplier lockButtonSupplier){
        this.swerveChassis = swerveChassis;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.zSupplier = zSupplier;
        this.lockButtonSupplier = lockButtonSupplier;
        this.fieldOriented = fieldOriented;
        
        addRequirements(swerveChassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){

      
        
        SwerveModuleState[] states;

        // ALTERING VALUES

        //Joystick values -> double
        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();
        double zSpeed = zSupplier.getAsDouble();

        boolean lockButton = lockButtonSupplier.getAsBoolean();

        SmartDashboard.putNumber("z speed", zSpeed);
     
        //apply deadzone to speed values  //ignore small input values that may be due to noise or slight, unintended movements.
        xSpeed = deadzone(xSpeed); 
        ySpeed = deadzone(ySpeed); 
        zSpeed = deadzone(zSpeed); 
    
        //square the speed values to make smoother acceleration 
        xSpeed = modifyAxis(xSpeed);  
        ySpeed = modifyAxis(ySpeed); 
        zSpeed = modifyAxis(zSpeed); 
    
        // SETTING SWERVE STATES
        if (fieldOriented) {
          states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
              ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveChassis.getRotation2d())); 
        } else {
          states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
              new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        }
        swerveChassis.setModuleStates(states);
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        swerveChassis.stopModules();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
    
    // ADDED METHODS
    public double deadzone(double num){
        return Math.abs(num) > 0.1 ? num : 0;
    }
    
    private static double modifyAxis(double num) {
      // Square the axis
      num = Math.copySign(num * num, num);
    
      return num;
    }
}
