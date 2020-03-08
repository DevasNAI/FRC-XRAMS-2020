/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public final class Constants {


    public final class Xbox
    {

		public static final int Control1X = 0;


    }


    public static final class PuertosModuloSwerve
{
    // Módulos

    // Módulo 1 = Frontal Izquierda
      public static final int Modulo1Llanta = 3;
      public static final int Modulo1Giro = 4;
  
    // Módulo 2 = Trasero Derecha
      public static final int Modulo2Llanta = 6;
      public static final int Modulo2Giro = 5;

    // Módulo 3 = Trasero Izquierda
      public static final int Modulo3Llanta = 7;
      public static final int Modulo3Giro = 8;
    
    // Módulo 4 = Frontal Derecha
      public static final int Modulo4Llanta = 10;
      public static final int Modulo4Giro = 9;




      // Puertos Encoder de Módulo

      public static final int Cancoder1 = 20;
      public static final int Cancoder2 = 21;
      public static final int Cancoder3 = 22;
      public static final int Cancoder4 = 23;


      // Reversas Encoder

      public static final boolean Modulo1ReversaEncoderGiro = false;
      public static final boolean Modulo2ReversaEncoderGiro = false; //true
      public static final boolean Modulo3ReversaEncoderGiro = false;
      public static final boolean Modulo4ReversaEncoderGiro = false; //true

      public static final boolean Modulo1ReversaEncoderManejo = false;
      public static final boolean Modulo2ReversaEncoderManejo = true; //true
      public static final boolean Modulo3ReversaEncoderManejo = true;
      public static final boolean Modulo4ReversaEncoderManejo = false; //true



      // Distancia entre centro de llanta derecha e izquierda

      public static final double TrackWidth = .38;// .38

      // Distancia entre las llantas frontales y traseras del robot

      public static final double Llantabase = .38;




      public static final Translation2d IzquierdoFrontal = new Translation2d(Llantabase, TrackWidth);
      public static final Translation2d DerechoFrontal = new Translation2d(Llantabase, TrackWidth);
      public static final Translation2d IzquierdoTrasero = new Translation2d(Llantabase, TrackWidth);
      public static final Translation2d DerechoTrasero = new Translation2d(Llantabase, TrackWidth);


      public static final SwerveDriveKinematics KinematicsManejo = new SwerveDriveKinematics(
        IzquierdoFrontal,
        DerechoFrontal,
        IzquierdoTrasero,
        DerechoTrasero);

      public static final boolean ReversaGyro = false;


      public static final double VelocidadMaxMetrosporsegundo = .3;



    }

public static final class ConstantesSwerve
{

      public static final double SpeedAngularPorSegundoMaxima = (2 * Math.PI) / 10;
      public static final double AceleracionAngularRadianPorSegundoCuadradoMaxima = (2 * Math.PI) / 10;
      public static final int EncoderCPR = 1024;

      public static final double DiametroLlantaMetros = 0.1;
      public static final double DistanciaPorPulsoEncoderLlanta = (DiametroLlantaMetros * Math.PI) / (double) EncoderCPR;
      public static final double kTurningEncoderDistancePerPulse = (2* Math.PI); // / (double) EncoderCPR;


      public static final double kPManejoConstante = .0105;
      public static final double kPManejoGiro = 0.105; //.105

      public static final int timeoutms = 10;
      
    }


    public static final class ConstantesAuto
{
  
      public static final double VelociMaxMetrosporsegundo = .2;
      public static final double MaxAceleracionMetrosporSegundoCuadrado = .01;
      public static final double MaxVelocidadAngularRadianesPorSegundo = Math.PI;
      public static final double VelocidadAngularMaximaRadianesPorSegundoCuadrado = Math.PI;


      public static final double PXController = .1;
      public static final double kPYController = .1;
      public static final double kPThetaController = .1;


      public static final TrapezoidProfile.Constraints ThetaConstraintsControl = new TrapezoidProfile.Constraints(MaxVelocidadAngularRadianesPorSegundo, VelocidadAngularMaximaRadianesPorSegundoCuadrado);
  
    }

}
