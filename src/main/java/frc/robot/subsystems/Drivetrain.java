// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants.Mecanum;
import frc.robot.Constants.PIDFFs;
import frc.robot.Constants.PatherPID;
import frc.robot.Constants.Encoders;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

//Iniciamos Subsistema Chasis
public class Drivetrain extends SubsystemBase {
  //Declarar sparks chasis
  private SparkMax RearRight = new SparkMax(Mecanum.kRearRightID, MotorType.kBrushed);
  private SparkMax RearLeft = new SparkMax(Mecanum.kRearLeftID, MotorType.kBrushed);
  private SparkMax FrontRight = new SparkMax(Mecanum.kFrontRightID, MotorType.kBrushed);
  private SparkMax FrontLeft = new SparkMax(Mecanum.kFrontLeftID, MotorType.kBrushed);

  //Crear configuración sparks
  private SparkMaxConfig RearRightConfig = new SparkMaxConfig();
  private SparkMaxConfig RearLeftConfig = new SparkMaxConfig();
  private SparkMaxConfig FrontRightConfig = new SparkMaxConfig();
  private SparkMaxConfig FrontLeftConfig = new SparkMaxConfig();

  private Encoder FrontLeftEncoder = new Encoder(
    Encoders.kEncoderFL_A,
    Encoders.kEncoderFL_B,
    false,
    Encoder.EncodingType.k4X
  );
  private Encoder RearLeftEncoder = new Encoder(
    Encoders.kEncoderRL_A,
    Encoders.kEncoderRL_B,
    false,
    Encoder.EncodingType.k4X
  );
  private Encoder FrontRightEncoder = new Encoder(
    Encoders.kEncoderFR_A,
    Encoders.kEncoderFR_B,
    true, 
    Encoder.EncodingType.k4X
  );
  private Encoder RearRightEncoder = new Encoder(
    Encoders.kEncoderRR_A,
    Encoders.kEncoderRR_B,
    true, 
    Encoder.EncodingType.k4X
  );

    //Encoders Virtuales
  private EncoderSim
    eSimFL,
    eSimFR,
    eSimRL,
    eSimRR
    ;

  // PID por rueda
  private final PIDController pidFL = new PIDController(
    PIDFFs.kP_wheel, 
    PIDFFs.kI_wheel, 
    PIDFFs.kD_wheel
  );
  private final PIDController pidFR = new PIDController(
    PIDFFs.kP_wheel, 
    PIDFFs.kI_wheel, 
    PIDFFs.kD_wheel
  );
  private final PIDController pidRL = new PIDController(
    PIDFFs.kP_wheel, 
    PIDFFs.kI_wheel, 
    PIDFFs.kD_wheel
  );
  private final PIDController pidRR = new PIDController(
    PIDFFs.kP_wheel, 
    PIDFFs.kI_wheel, 
    PIDFFs.kD_wheel
  );

  // Feedforward por rueda (kS, kV, kA) — valores de ejemplo: debes tunear
  private final SimpleMotorFeedforward ffFL = new SimpleMotorFeedforward(
    PIDFFs.kS_wheel, 
    PIDFFs.kV_wheel, 
    PIDFFs.kA_wheel
  );
  private final SimpleMotorFeedforward ffFR = new SimpleMotorFeedforward(
    PIDFFs.kS_wheel, 
    PIDFFs.kV_wheel, 
    PIDFFs.kA_wheel
  );
  private final SimpleMotorFeedforward ffRL = new SimpleMotorFeedforward(
    PIDFFs.kS_wheel, 
    PIDFFs.kV_wheel, 
    PIDFFs.kA_wheel
  );
  private final SimpleMotorFeedforward ffRR = new SimpleMotorFeedforward(
    PIDFFs.kS_wheel, 
    PIDFFs.kV_wheel, 
    PIDFFs.kA_wheel
  );

  private MecanumDriveKinematics xRC_Kinematics;
  private MecanumDriveOdometry xRC_Odometry;
  private MecanumDrivePoseEstimator Waze;

  private final Field2d cancha = new Field2d();

  private AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  double PoseSimX = 0.0;
  double PoseSimY = 0.0;
  
  //Giroscopio Virtual
  AnalogGyroSim gSimNavX;
   
  private boolean fieldOriented = true;

  private ChassisSpeeds SimulatorSpeeds = new ChassisSpeeds();

  public Drivetrain() {
    //Configurar los Sparks
    RearRightConfig.inverted(false).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40).voltageCompensation(Mecanum.kMaxVolts);
    RearLeftConfig.inverted(true).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40).voltageCompensation(Mecanum.kMaxVolts);
    FrontRightConfig.inverted(false).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40).voltageCompensation(Mecanum.kMaxVolts);
    FrontLeftConfig.inverted(true).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40).voltageCompensation(Mecanum.kMaxVolts);

    //Configurar Sparks
    RearRight.configure(
      RearRightConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );
    RearLeft.configure(
      RearLeftConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );
    FrontRight.configure(
      FrontRightConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );
    FrontLeft.configure(
      FrontLeftConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );

    //Configurar Encoders
    FrontLeftEncoder.setSamplesToAverage(10);
    FrontLeftEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    FrontLeftEncoder.setMinRate(10);
    FrontLeftEncoder.reset();

    RearLeftEncoder.setSamplesToAverage(10);
    RearLeftEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    RearLeftEncoder.setMinRate(10);
    RearLeftEncoder.reset();

    FrontRightEncoder.setSamplesToAverage(10);
    FrontRightEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    FrontRightEncoder.setMinRate(10);
    FrontRightEncoder.reset();

    RearRightEncoder.setSamplesToAverage(10);
    RearRightEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    RearRightEncoder.setMinRate(10);
    RearRightEncoder.reset();

    //Configurar VirtualCoders y VirtualGyro
    if (RobotBase.isSimulation()) {
      eSimFL = new EncoderSim(FrontLeftEncoder);
      eSimFR = new EncoderSim(FrontRightEncoder);
      eSimRL = new EncoderSim(RearLeftEncoder);
      eSimRR = new EncoderSim(RearRightEncoder);
      gSimNavX = new AnalogGyroSim(0); 
    }

    //Ubicacion llantas y giroscopio
    Translation2d frontLeftLocation = new Translation2d(0.258, 0.288);
    Translation2d frontRightLocation = new Translation2d(0.258, -0.288);
    Translation2d rearLeftLocation = new Translation2d(-0.258, 0.288);
    Translation2d rearRightLocation = new Translation2d(-0.258, -0.288);

    Pose2d initialPose = new Pose2d(0, 0, getHeading());


    //Declarar posiciones de las llantas al iniciar
    MecanumDriveWheelPositions initialWheelPositions = new MecanumDriveWheelPositions(
      FrontLeftEncoder.getDistance(),
      FrontRightEncoder.getDistance(),
      RearLeftEncoder.getDistance(),
      RearRightEncoder.getDistance()
    );

    //Declarar Cinematicas, Odometria y estimacion de posicion
    xRC_Kinematics = new MecanumDriveKinematics(
      frontLeftLocation, 
      frontRightLocation, 
      rearLeftLocation, 
      rearRightLocation
    );

    xRC_Odometry = new MecanumDriveOdometry(
      xRC_Kinematics, 
      getHeading(), 
      initialWheelPositions, 
      initialPose
    );

    Waze = new MecanumDrivePoseEstimator(
      xRC_Kinematics, 
      getHeading(), 
      initialWheelPositions, 
      initialPose,
      VecBuilder.fill(0.1,0.25,0.01),
      VecBuilder.fill(0.5,0.5,99.0)
    );

    navx.reset();

    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose2d, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveWithSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(
          PatherPID.kP_PatherTrans,
          PatherPID.kI_PatherTrans,
          PatherPID.kD_PatherTrans
        ),
        new PIDConstants(
          PatherPID.kP_PatherRot,
          PatherPID.kI_PatherRot,
          PatherPID.kD_PatherRot) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
    this // Reference to this subsystem to set requirements
  );

    SmartDashboard.putBoolean("FOD", fieldOriented);
    SmartDashboard.putData("Navx Angle", navx);
    SmartDashboard.putData("Field", cancha);
  }

  public void DriveCartesianWithAngles(double MecanumMove, double MecanumStrafe, double MecanumRotacionPID){
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      MecanumMove * Mecanum.kMaxSpeedWheel, 
      MecanumStrafe * -(Mecanum.kMaxSpeedWheel), 
      MecanumRotacionPID * -4.5
    );

    driveWithSpeeds(chassisSpeeds);
  }

  public void driveWithSpeeds(ChassisSpeeds chassisSpeeds){
 
    SimulatorSpeeds = chassisSpeeds;

    //Leer velocidades de las llantas y desaturar para no pasar el maximo
    MecanumDriveWheelSpeeds wheelSpeeds = xRC_Kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(Mecanum.kMaxSpeedWheel);

    //Velocidad Meta en M/s (A cuanto queremos ir)
    double MetaFL = wheelSpeeds.frontLeftMetersPerSecond;
    double MetaFR = wheelSpeeds.frontRightMetersPerSecond;
    double MetaRL = wheelSpeeds.rearLeftMetersPerSecond;
    double MetaRR = wheelSpeeds.rearRightMetersPerSecond;
    
    //Velocidad Actual en M/s (A cuanto realmente vamos)
    double RealFL = FrontLeftEncoder.getRate();
    double RealFR = FrontRightEncoder.getRate();
    double RealRL = RearLeftEncoder.getRate();
    double RealRR = RearRightEncoder.getRate();

    //PID (Unitless por si solo)
    double FL_PID = pidFL.calculate(RealFL, MetaFL);
    double FR_PID = pidFR.calculate(RealFR, MetaFR);
    double RL_PID = pidRL.calculate(RealRL, MetaRL);
    double RR_PID = pidRR.calculate(RealRR, MetaRR);

    //FeedForward (En VOLTS)
    double FL_FF = ffFL.calculate(MetaFL);
    double FR_FF = ffFR.calculate(MetaFR);
    double RL_FF = ffRL.calculate(MetaRL);
    double RR_FF = ffRR.calculate(MetaRR);

    //Combinacion (VOLTS)
    double VoltsFL = FL_FF + FL_PID;
    double VoltsFR = FR_FF + FR_PID;
    double VoltsRL = RL_FF + RL_PID;
    double VoltsRR = RR_FF + RR_PID;

    //Ajustar limites de voltaje al motor
    double PercentFL = MathUtil.clamp(VoltsFL, -(Mecanum.kMaxVolts), Mecanum.kMaxVolts);
    double PercentFR = MathUtil.clamp(VoltsFR, -(Mecanum.kMaxVolts), Mecanum.kMaxVolts);
    double PercentRL = MathUtil.clamp(VoltsRL, -(Mecanum.kMaxVolts), Mecanum.kMaxVolts);
    double PercentRR = MathUtil.clamp(VoltsRR, -(Mecanum.kMaxVolts), Mecanum.kMaxVolts);

    //Establecer control de motores ya con PID a otra variable mas logica
    double OutputFL = PercentFL;
    double OutputFR = PercentFR;
    double OutputRL = PercentRL;
    double OutputRR = PercentRR;

    //Mandar el dato a cada motor
    FrontLeft.setVoltage(OutputFL);
    FrontRight.setVoltage(OutputFR);
    RearLeft.setVoltage(OutputRL);
    RearRight.setVoltage(OutputRR);

    //Publicar velocidades meta a 
    SmartDashboard.putNumber("ROD_TargetSpeeds/Meta_FL", MetaFL);
    SmartDashboard.putNumber("ROD_TargetSpeeds/Meta_FR", MetaFR);
    SmartDashboard.putNumber("ROD_TargetSpeeds/Meta_RL", MetaRL);
    SmartDashboard.putNumber("ROD_TargetSpeeds/Meta_RR", MetaRR);
    }
  

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      FrontLeftEncoder.getRate(),
      FrontRightEncoder.getRate(),
      RearLeftEncoder.getRate(),
      RearRightEncoder.getRate()
    );
  }

  public void setFieldOriented(boolean enabled) {
    fieldOriented = enabled;
  }

  public boolean isFieldOriented() {
    return fieldOriented;
  }

  public void stopMotors() {
    FrontLeft.setVoltage(0);
    FrontRight.setVoltage(0);
    RearLeft.setVoltage(0);
    RearRight.setVoltage(0);
  }

  public Rotation2d getRotation2d() {
    return navx.getRotation2d();
  }

  public void resetNavx() {
    if (RobotBase.isReal()){
      navx.reset();
    }
    else{
      gSimNavX.resetData();
    }
  }

  public double getNavXDegrees(){
    return navx.getAngle();
  }

  public double getHeadingDegrees(){
    if (RobotBase.isReal()){
      return (-navx.getAngle());
    }
    else{
      return gSimNavX.getAngle();
    }
  }

  public double getHeadingRadians(){
    return Radians.convertFrom(getHeadingDegrees(), Degrees);
  }

  public Rotation2d getHeading(){
    if (RobotBase.isReal()){
      return Rotation2d.fromDegrees(-navx.getAngle());
    }
    else{
      return Rotation2d.fromDegrees(gSimNavX.getAngle());
    }
  }

  public Pose2d getPose() {
    return xRC_Odometry.getPoseMeters();
  }

  public Pose2d getPose2d(){
    return Waze.getEstimatedPosition();
  }

  public void resetEncoders() {
    FrontLeftEncoder.reset();
    FrontRightEncoder.reset();
    RearLeftEncoder.reset();
    RearRightEncoder.reset();
  }

  public ChassisSpeeds getChassisSpeeds(){
    MecanumDriveWheelSpeeds currentWheelSpeeds = getWheelSpeeds();
    return xRC_Kinematics.toChassisSpeeds(currentWheelSpeeds);
  }

  public MecanumDriveWheelPositions getCurrentWheelPositions(){
    return new MecanumDriveWheelPositions(
      FrontLeftEncoder.getDistance(),
      FrontRightEncoder.getDistance(),
      RearLeftEncoder.getDistance(),
      RearRightEncoder.getDistance()
    );
  }

  public void resetOdometry(Pose2d pose){
    xRC_Odometry.resetPosition(getHeading(), getCurrentWheelPositions(), pose);
    Waze.resetPosition(getHeading(), getCurrentWheelPositions(), pose);
  }

  public void ResetALL(){
    resetNavx();
    resetOdometry(new Pose2d(0,0, getHeading()));
    resetEncoders();
  }
   
  public void addVisionMeasurement(Pose2d visionPose2d, double timestampSeconds) {
      Waze.addVisionMeasurement(visionPose2d, timestampSeconds);
  }


  @Override
  public void periodic() {
    MecanumDriveWheelPositions wheelPositions = getCurrentWheelPositions();

    //Actualiza la Odometria
    xRC_Odometry.update(getHeading(), wheelPositions);

    //Actualizar estimador de pose
    Waze.update(getHeading(), wheelPositions);

    //Proyectamos el robot en la cancha virtual de la UI
    cancha.setRobotPose(xRC_Odometry.getPoseMeters());  


     //------------------------Publicar valores utiles------------------------------

    //Pose XY del robot
    SmartDashboard.putNumber("RobotPose/Pose X (m)", getPose2d().getX());
    SmartDashboard.putNumber("RobotPose/Pose Y (m)", getPose2d().getY());

    //Proyectar la cancha
    SmartDashboard.putData("Cancha", cancha);

    //--------------------------Telemetria teleop-------------------------------

    //Angulo actual y meta
    SmartDashboard.putNumber("AngulosChasis/Heading", getHeadingDegrees());
    
    //Velocidades Reales
    SmartDashboard.putNumber("RealSpeeds/Real_FL", FrontLeftEncoder.getRate());
    SmartDashboard.putNumber("RealSpeeds/Real_FR", FrontRightEncoder.getRate());
    SmartDashboard.putNumber("RealSpeeds/Real_RL", RearLeftEncoder.getRate());
    SmartDashboard.putNumber("RealSpeeds/Real_RR", RearRightEncoder.getRate());
  }


  @Override
  public void simulationPeriodic() {
    //Verificar que los objetos de simulacion esten inicializados
    if (
      eSimFL == null || 
      eSimFR == null || 
      eSimRL == null || 
      eSimRR == null || 
      gSimNavX == null||
      SimulatorSpeeds == null
    ) 
      {
        return; 
    }

    MecanumDriveWheelSpeeds simWheelSpeeds = xRC_Kinematics.toWheelSpeeds(SimulatorSpeeds);
    //Velocidades de las ruedas para simulacion (en M/s)
    double VeloSimFL = simWheelSpeeds.frontLeftMetersPerSecond;
    double VeloSimFR = simWheelSpeeds.frontRightMetersPerSecond;
    double VeloSimRL = simWheelSpeeds.rearLeftMetersPerSecond;
    double VeloSimRR = simWheelSpeeds.rearRightMetersPerSecond;

    //Inyectar velocidades a los encoders virtuales
    eSimFL.setRate(VeloSimFL);
    eSimFL.setDistance(eSimFL.getDistance() + (VeloSimFL * 0.02));

    eSimFR.setRate(VeloSimFR);
    eSimFR.setDistance(eSimFR.getDistance() + (VeloSimFR * 0.02));

    eSimRL.setRate(VeloSimRL);
    eSimRL.setDistance(eSimRL.getDistance() + (VeloSimRL * 0.02));

    eSimRR.setRate(VeloSimRR);
    eSimRR.setDistance(eSimRR.getDistance() + (VeloSimRR * 0.02));

    //Angulo del giroscopio para simulacion y convertirlo a grados
    double DeltaDegrees = SimulatorSpeeds.omegaRadiansPerSecond * 0.02 * (180.0 / Math.PI);
    
    //Actualizar el VirtualGyro
    gSimNavX.setAngle(gSimNavX.getAngle() + DeltaDegrees);

    Pose2d currentPose = getPose2d();

    //Actualizar la pose del robot en la simulacion
    cancha.setRobotPose(currentPose);

    //Publicar Veldades y Poses a Dashboard
    SmartDashboard.putNumber("SimSpeeds/SimSpeedX", SimulatorSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SimSpeeds/SimSpeedY", SimulatorSpeeds.vyMetersPerSecond);
  }
}