/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.PuertosModuloSwerve;

public class SubsistemaDrivetrain extends SubsystemBase 

{
  // Se declara el Giroscopio NavX

  public final AHRS Giroscopio = new AHRS(SPI.Port.kMXP);

  // public SwerveDriveOdometry odometria = new SwerveDriveOdometry(PuertosModuloSwerve.KinematicsManejo,
  // getAngle(), new Pose2d(2, 13.5, new Rotation2d()));    Esta es una declaración sobre otro tipo de constructor, no se ha usado

  //  Se declara odometria
  public SwerveDriveOdometry odometria = new SwerveDriveOdometry(PuertosModuloSwerve.KinematicsManejo,
  getAngle());


  
  //  Se declaran los módulos, del 1 al 4 iniciando del frontal izquierdo.

  public  final SwerveModule Lfrontal = new SwerveModule(
      PuertosModuloSwerve.Modulo1Llanta,
      PuertosModuloSwerve.Modulo1Giro,
      PuertosModuloSwerve.Cancoder1,
      PuertosModuloSwerve.Modulo1ReversaEncoderManejo,
      PuertosModuloSwerve.Modulo1ReversaEncoderGiro);

  public final SwerveModule Rtrasero = new SwerveModule(
      PuertosModuloSwerve.Modulo2Llanta,
      PuertosModuloSwerve.Modulo2Giro,
      PuertosModuloSwerve.Cancoder2,
      PuertosModuloSwerve.Modulo2ReversaEncoderManejo,
      PuertosModuloSwerve.Modulo2ReversaEncoderGiro);

  public final SwerveModule Ltrasero = new SwerveModule(
      PuertosModuloSwerve.Modulo3Llanta,
      PuertosModuloSwerve.Modulo3Giro,
      PuertosModuloSwerve.Cancoder3,
      PuertosModuloSwerve.Modulo3ReversaEncoderManejo,
      PuertosModuloSwerve.Modulo3ReversaEncoderGiro);

  public final SwerveModule Rfrontal = new SwerveModule(
      PuertosModuloSwerve.Modulo4Llanta,
      PuertosModuloSwerve.Modulo4Giro,
      PuertosModuloSwerve.Cancoder4,
      PuertosModuloSwerve.Modulo4ReversaEncoderManejo,
      PuertosModuloSwerve.Modulo4ReversaEncoderGiro);



  public SubsistemaDrivetrain()
  {
    // Aquí intento posicionar los encoders en 0
    Giroscopio.resetDisplacement();
    resetEncoders();

  }
  


  public Rotation2d getAngle()
  {
    return Rotation2d.fromDegrees(Giroscopio.getAngle() * (PuertosModuloSwerve.ReversaGyro ? 1.0 : -1.0));  
    //  Sí el giro es reversa true, el valor será 1, falso es -1
  }
 
  @Override
  public void periodic()
  {
    // radianes

    odometria.update(new Rotation2d(getHeading()),
    Lfrontal.getState(),
    Rfrontal.getState(),
    Ltrasero.getState(),
    Rtrasero.getState());
    
    /**
     * El SmartDashboard es una aplicación de apoyo donde podemos imprimir los distintos valores que deseemos
     * tales como el la posición actual de los encoders o el giroscopio
     */
    SmartDashboard.putNumber("Encoder Modulo 1 ", Lfrontal.EncoderModulo.getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Modulo 4 ", Rfrontal.EncoderModulo.getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Modulo 3 ", Ltrasero.EncoderModulo.getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Modulo 2 ", Rtrasero.EncoderModulo.getAbsolutePosition());


    
    setDefaultCommand(RobotContainer.ComandoSwerve);
  }
/*
  public Pose2d getPose2d()
  {
    return odometria.getPoseMeters(); //  Regresa la posición estimada del robot
  }
*/

  public void resetOdometry(Pose2d pose)
  {
    odometria.resetPosition(pose, getAngle());
  }


  /** 
  *Método para usar el robot con el joystick
  *
  * @param xSpeed            velocidad en la dirección x (adelante)
  * @param ySpeed            velocidad en la dirección y (lados)
  * @param rot               grado angular del robot
  * @param fieldRelative     Los valores de x o y son relativos al campo
  *
  */
  @SuppressWarnings("ParameterName")
  public void Manejo(double xSpid, double ySpid, double rot, boolean RelativoalCampo)
  {
    // var estadosModuloSwerve = PuertosModuloSwerve.KinematicsManejo   En esta variable se le agrega una condición en caso de ser relativo al campo
    //     .toSwerveModuleStates(
    //   RelativoalCampo ? ChassisSpeeds.fromFieldRelativeSpeeds(
    //     xSpid, ySpid, rot, getAngle())
    //                         : new ChassisSpeeds(xSpid, ySpid, rot));


   var estadosModuloSwerve = PuertosModuloSwerve.KinematicsManejo
        .toSwerveModuleStates( new ChassisSpeeds(xSpid, ySpid, rot));    
                            
    if(ySpid >= .3)
    {
      ySpid = .4;
    }
    else if(ySpid <= -.3)
    {
      ySpid = -.4;
    }   
    else{
      ySpid = 0;
    }

    // SwerveDriveKinematics.normalizeWheelSpeeds(estadosModuloSwerve,
    //     PuertosModuloSwerve.VelocidadMaxMetrosporsegundo );

    // Lfrontal.setEstadoDeseado(estadosModuloSwerve[0]);


    // Lfrontal.setEstadoDeseado(estadosModuloSwerve[0]);  En estos comentarios está la representación de cómo se veía anteriormente.
    // Rfrontal.setEstadoDeseado(estadosModuloSwerve[1]);
    // Ltrasero.setEstadoDeseado(estadosModuloSwerve[2]);
    // Rtrasero.setEstadoDeseado(estadosModuloSwerve[3]);

    Lfrontal.setEstadosdeModulo(estadosModuloSwerve, estadosModuloSwerve[0], xSpid, ySpid);
    Rfrontal.setEstadosdeModulo(estadosModuloSwerve, estadosModuloSwerve[1], xSpid, ySpid);
    Ltrasero.setEstadosdeModulo(estadosModuloSwerve, estadosModuloSwerve[2], xSpid, ySpid);
    Rtrasero.setEstadosdeModulo(estadosModuloSwerve, estadosModuloSwerve[3], xSpid, ySpid);

    

    



  }





// Recibe el estado del módulo de swerve

  

 
// Se supone que reinicia el encoder pero no creo que jale

  public void resetEncoders()
  {

    Lfrontal.ReiniciarEncoders();
    Rfrontal.ReiniciarEncoders();
    Ltrasero.ReiniciarEncoders();
    Rtrasero.ReiniciarEncoders();

  }

  // Hace que el heading del robot sea 0, mirar hacia la posición 0

  public void zeroHeading()

  {
  

    Giroscopio.resetDisplacement();
    Giroscopio.reset();


  }

  // Regresa el heading del robot


  // Regresa el valor de giro del robot
  public double getRangodeGiro()
  {
    return Giroscopio.getRate() * (PuertosModuloSwerve.ReversaGyro ? -1.0 : 1.0);
  }

  public double getHeading()
  {
    return Math.IEEEremainder(Giroscopio.getAngle(), 360 * (PuertosModuloSwerve.ReversaGyro ? -1.0 : 1.0));
  }





}

















/*
Converting module states to chassis speeds
One can also use the kinematics object to convert an array of SwerveModuleState objects to a singular ChassisSpeeds object.
The toChassisSpeeds(SwerveModuleState... states) (Java) / ToChassisSpeeds(SwerveModuleState... states) (C++) method can be used to achieve this.






// Example module states
var frontLeftState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-140.19));
var frontRightState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-39.81));
var backLeftState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-109.44));
var backRightState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-70.56));

// Convert to chassis speeds
ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(
  frontLeftState, frontRightState, backLeftState, backRightState);

// Getting individual speeds
double forward = chassisSpeeds.vxMetersPerSecond;
double sideways = chassisSpeeds.vyMetersPerSecond;
double angular = chassisSpeeds.omegaRadiansPerSecond;
*/