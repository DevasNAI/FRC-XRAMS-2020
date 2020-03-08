/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ConstantesSwerve;
import frc.robot.Constants.PuertosModuloSwerve;


public class SwerveModule extends SubsystemBase {

    public WPI_TalonFX Llanta;
    public WPI_TalonFX Giro;
    public CANCoder EncoderModulo;

    // Los Configuration permiten acceder a las configuraciones de los dispositivos de CTRE como constantes que la libreria tiene

    public CANCoderConfiguration Configuracion;
    public TalonFXConfiguration Configurachon;
  
  
   
    
  
   // Controlador PID para controlar el giro del módulo individual

    private final PIDController PIDGiro = new PIDController(ConstantesSwerve.kPManejoGiro, 0, 0);

    //  En este ProfiledPIDController se supone que le da un giro más suave al módulo
    // private final ProfiledPIDController PIDLlanta = 
    // new ProfiledPIDController(
    //   ConstantesSwerve.kPManejoConstante, .001, 0,
    // new TrapezoidProfile.Constraints(
    //   ConstantesSwerve.SpeedAngularPorSegundoMaxima,
    //   ConstantesSwerve.AceleracionAngularRadianPorSegundoCuadradoMaxima));
  
    // Controlador PID para regular la velocidad de la llanta
    private final PIDController PIDLlanta = new PIDController(ConstantesSwerve.kPManejoConstante, 0, 0);
  
    
  
  
    /**
     * Este es un constructor de módulo de swerve, inicializo los motores de giro y de manejo(llanta) para poder declarar un módulo swerve
     * @param motorLlanta Motor conectado a la transmisión que gira la llanta
     * @param motorangulo Motor conectado a la transmisión que gira la posición de la llanta (el módulo)
     * @param encoderModulo Encoder magnético que se encuentra en el módulo
     * @param InversionEncoderLlanta  Booleano que invierte la dirección del motor de la llanta
     * @param InversionEncoderGiro    Booleano que invierte la dirección del motor de giro
     */
    public SwerveModule(int motorLlanta, int motorangulo, int encoderModulo, boolean InversionEncoderLlanta,
        boolean InversionEncoderGiro)
  
    {
      Llanta = new WPI_TalonFX(motorLlanta);
      Giro = new WPI_TalonFX(motorangulo);
      // EncoderModulo = new CANCoder(encoderModulo);
      EncoderModulo = new CANCoder(encoderModulo);
  
      Configuracion = new CANCoderConfiguration();
      Configurachon = new TalonFXConfiguration();
  
  
    
      // Le da valores de fábrica a los motores y encoder
      Llanta.configFactoryDefault();
      EncoderModulo.configFactoryDefault();
      Giro.configFactoryDefault();
  
      // Obtiene la posición del encoder
      EncoderModulo.getAbsolutePosition();
      Llanta.setInverted(InversionEncoderLlanta);
      Llanta.getSelectedSensorPosition(); // Toma el encoder del motor de llanta
  
      EncoderModulo.configAbsoluteSensorRange(Configuracion.absoluteSensorRange);
  
      // EncoderModulo.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
  
      EncoderModulo.setPosition(ConstantesSwerve.kTurningEncoderDistancePerPulse);  //ConstantesSwerve.kTurningEncoderDistancePerPulse
      
      //  Le da un input máximo y minimo al PID
      PIDGiro.enableContinuousInput(-Math.PI, Math.PI);
  
      
      
  
      EncoderModulo.configSensorInitializationStrategy(Configuracion.initializationStrategy);
      EncoderModulo.getAllConfigs(Configuracion, ConstantesSwerve.timeoutms);
  
  
      
      // Configurachon.slot0.kP = ConstantesSwerve.kPManejoGiro;
  
      Llanta.configNominalOutputForward(0, ConstantesSwerve.timeoutms);
      Llanta.configNominalOutputReverse(0, ConstantesSwerve.timeoutms);
  
      Llanta.configPeakOutputForward(.8, ConstantesSwerve.timeoutms);
      Llanta.configPeakOutputReverse(-.8, ConstantesSwerve.timeoutms);
  
  
      Giro.configNominalOutputForward(0,ConstantesSwerve.timeoutms);
      Giro.configNominalOutputReverse(0,ConstantesSwerve.timeoutms);
  
      Giro.configPeakOutputForward(.8, ConstantesSwerve.timeoutms);
      Giro.configPeakOutputReverse(-.8, ConstantesSwerve.timeoutms);
  
      // Giro.configMotionCruiseVelocity(15000,ConstantesSwerve.timeoutms);
      // Giro.configMotionAcceleration(16000, ConstantesSwerve.timeoutms);
  
      // Giro.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      // 0, ConstantesSwerve.timeoutms);
  
  
      // Giro.setNeutralMode(NeutralMode.Brake);
  
    }
  
  
    public SwerveModuleState getState()
    {
      return new SwerveModuleState(Llanta.getSelectedSensorPosition(), new Rotation2d(EncoderModulo.getAbsolutePosition()));
    }
  



  /**
   * En este método utilizo estado para poder darle un estado a mi módulo de swerve, anteriormente estaba todo en comillas
   *  porque no funcionaba y quiero que mis motores reaccionen conforme al joystick, cosa que no realizaba.
   * @param estado  Estado actual del módulo
   */
    public void setEstadoDeseado(SwerveModuleState estado)
    {
      final var SalidaManejo = PIDLlanta.calculate(Llanta.getSelectedSensorPosition(0), estado.speedMetersPerSecond);
      
      final var SalidaGiro = PIDGiro.calculate(EncoderModulo.getAbsolutePosition(), estado.angle.getDegrees() ); 
      // final var SalidaGiro = MathUtil.clamp(PIDGiro.calculate(EncoderModulo.getAbsolutePosition(), estado.angle.getDegrees()), -.8, .8);
      
      SmartDashboard.putNumber("SalidaGiroPID " + " ", SalidaGiro);
  
      // Se le da el valor entre -1 y 1 para mover el motor, el set de los controladores de CTRE utiliza un módo y un valor,
      // el modo en este caso es el porcentaje de salida y el porcentaje de salida serán las variables de salidamanejo y salidagiro
      Llanta.set(ControlMode.PercentOutput, SalidaManejo);
  
      Giro.set(TalonFXControlMode.PercentOutput, SalidaGiro);

  
    }
  
  /**
   * En este método trato de para darle la posición a los módulos de swerve.
   * @param estadosModuloSwerve Es el estado actual del módulo de swerve y la posición en la que debe estar
   * @param estado  Me permite utilizar las funciones de SwerveModuleState
   * @param spi     Valor para motor de llanta, un joystick entre -1 y 1
   * @param izq     Valor para motores de módulo de giro
   */
    public void setEstadosdeModulo(SwerveModuleState[] estadosModuloSwerve,  SwerveModuleState estado, double spi, double izq)
    {
      
      final var SalidaGiro = MathUtil.clamp(PIDGiro.calculate(EncoderModulo.getAbsolutePosition(), estado.angle.getDegrees()), -.8, .8);
      
      final var SalidaManejo = PIDLlanta.calculate(Llanta.getSelectedSensorPosition(), estado.speedMetersPerSecond);
  
      var IzqFrontalState = new SwerveModuleState(SalidaGiro, Rotation2d.fromDegrees(124.4)); // 124.4
      var DerFrontalState = new SwerveModuleState(SalidaGiro, Rotation2d.fromDegrees(241));  //-120
      var IzqTraseroState = new SwerveModuleState(SalidaGiro, Rotation2d.fromDegrees(271.05)); // -85, 273.2
      var DerTraseroState = new SwerveModuleState(SalidaGiro, Rotation2d.fromDegrees(287.4));  //  -74.23, 285.38
  
  
      ChassisSpeeds chassisSpids = PuertosModuloSwerve.KinematicsManejo.toChassisSpeeds(
        IzqFrontalState, DerFrontalState, IzqTraseroState, DerTraseroState);
  
      double adelante = chassisSpids.vxMetersPerSecond;
  
      double lado = chassisSpids.vyMetersPerSecond;
      double strafe = chassisSpids.omegaRadiansPerSecond;
      
  
      SmartDashboard.putNumber("SalidaGiroPID " + " ", SalidaGiro);
      SmartDashboard.putNumber("SalidaLlantaPID " + " ", SalidaManejo);
  
  
      Giro.set(TalonFXControlMode.PercentOutput, lado, DemandType.ArbitraryFeedForward, izq);
      // Llanta.set(TalonFXControlMode.PercentOutput, adelante, DemandType.ArbitraryFeedForward, strafe);
      Llanta.set(TalonFXControlMode.PercentOutput, spi);

      /*
      SwerveDriveKinematics.normalizeWheelSpeeds(estadosdeseados, PuertosModuloSwerve.VelocidadMaxMetrosporsegundo);

      */
    }
    
  
  
    public void ReiniciarEncoders()
    {
      // Llanta.configRemoteSensorClosedLoopDisableNeutralOnLOS(true, 1);
      // Giro.configRemoteSensorClosedLoopDisableNeutralOnLOS(true, 1);
     
      EncoderModulo.configMagnetOffset(Configuracion.magnetOffsetDegrees);
      Llanta.getSelectedSensorPosition(0);
      Giro.getSelectedSensorPosition(0);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }


}