// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {

  public SwerveModule[] mSwerveMods;
  public AHRS gyro;

  private SwerveModulePosition[] positions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  private final SwerveDriveOdometry odometer;

  /** Creates a new Swerve. */
  public Swerve() {

    gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    mSwerveMods = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.Mod0.constants),
      new SwerveModule(1, SwerveConstants.Mod1.constants),
      new SwerveModule(2, SwerveConstants.Mod2.constants),
      new SwerveModule(3, SwerveConstants.Mod3.constants)
    };

    odometer = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, new Rotation2d(), positions);
    
        String title = "Swerve";



    for(SwerveModule mod : mSwerveMods){
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
        Shuffleboard.getTab(title).addNumber("Mod " + mod.moduleNumber + " CANcoder", () -> mod.getCANcoder().getDegrees());
        Shuffleboard.getTab(title).addNumber("Mod " + mod.moduleNumber + " Angle", ()-> mod.getPosition().angle.getDegrees());
        Shuffleboard.getTab(title).addNumber("Mod " + mod.moduleNumber + " Velocity", ()-> mod.getState().speedMetersPerSecond);
        Shuffleboard.getTab(title).addNumber("Mod " + mod.moduleNumber + "setAngle",()-> mod.getDesiredState());
    }

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    
    SwerveModuleState[] swerveModuleStates =
      SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            translation.getX(), 
                            translation.getY(), 
                            rotation, 
                            getHeading()
                        )
                        : new ChassisSpeeds(
                            translation.getX(), 
                            translation.getY(), 
                            rotation)
                        );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);


    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }
  public void stopDriving() {
    for(SwerveModule mod : mSwerveMods) {
        mod.stopDriving();
    }
  }
  
  /**
   * Resets the odometer value
   */
  public void resetOdometry(Pose2d pose2d){
    System.out.println("Reset Odometry: " + pose2d.getX() + ", " + pose2d.getY());
    this.positions[0] = new SwerveModulePosition();
    this.positions[1] = new SwerveModulePosition();
    this.positions[2] = new SwerveModulePosition();
    this.positions[3] = new SwerveModulePosition();
    odometer.resetPosition(getGyroYaw(), positions, pose2d);
  }


  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  public Pose2d getPose() {   
    return odometer.getPoseMeters();
  }

  public Rotation2d getHeading(){
    return getPose().getRotation();
}

  public Rotation2d getGyroYaw() {
    return gyro.getRotation2d();
  }

  public void resetModulesToAbsolute(){
    for(SwerveModule mod : mSwerveMods){
        mod.resetToAbsolute();
    }
  }

  public void zeroHeading(){
    odometer.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getGyroYaw(), getModulePositions());

    
    

  }

}
