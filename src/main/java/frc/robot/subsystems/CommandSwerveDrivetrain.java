package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.CoralLevels;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.CurrentLimits;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.vision.VisonAdjustment;

import frc.robot.vision.VisonAdjustment;

import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  private final SwerveRequest.FieldCentric m_FieldCentricdrive =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.RobotCentric m_RobotCentricdrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private SlewRateLimiter m_xLimiter = new SlewRateLimiter(2.5);
  private SlewRateLimiter m_yLimiter = new SlewRateLimiter(2.5);
  private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(3);
  private PIDController m_xController = new PIDController(TunerConstants.visonXAdjustmentP, TunerConstants.visonXAdjustmentI, TunerConstants.visonXAdjustmentD);
  private PIDController m_yController = new PIDController(TunerConstants.visonYAdjustmentP, TunerConstants.visonYAdjustmentI, TunerConstants.visonYAdjustmentD);

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final SwerveRequest m_brake = new SwerveRequest.SwerveDriveBrake();



  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  private double m_last_speed = 0.0;

  private double m_max_speed = 0.0;
  private double m_max_accel = 0.0;

  public Pose2d getPose2d() {
    return getState().Pose;
  }

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    applySupplyCurrentLimits();
    configureAutobuilder();
    setupVisonPIDs();
    setupVisonPIDs();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    applySupplyCurrentLimits();
    configureAutobuilder();
    setupVisonPIDs();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    applySupplyCurrentLimits();
    configureAutobuilder();
    setupVisonPIDs();
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }

    SwerveDriveState state = getState();

    double cur_accel = state.ModuleStates[1].speedMetersPerSecond - m_last_speed;
    SmartDashboard.putNumber("Accel", cur_accel);
    if (cur_accel > m_max_accel) {
      m_max_accel = cur_accel;
    }

    if (state.ModuleStates[1].speedMetersPerSecond > m_max_speed) {
      m_max_speed = state.ModuleStates[1].speedMetersPerSecond;
    }

    SmartDashboard.putNumber("Max Speed", m_max_speed);
    SmartDashboard.putNumber("Max Accel", m_max_accel);

    SmartDashboard.putBoolean("Can Vison Align", VisonAdjustment.hasTarget());
    SmartDashboard.putBoolean("Vison PIDs Aligned", visonPIDsAtSetpoint());

    SmartDashboard.putBoolean("Running Limelight", false);

    m_last_speed = state.ModuleStates[0].speedMetersPerSecond;
  }

  private void setupVisonPIDs() {
    m_xController.setTolerance(TunerConstants.visonXErrorTolerance);
    m_yController.setTolerance(TunerConstants.visonYErrorTolerance);
  }

  public Command gasPedalCommand(
      Supplier<Double> fieldCentricthrottleSupplier,
      Supplier<Double> robotCentricthrottleSupplier,
      Supplier<Double> rotationSupplier,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<CoralLevels> level) {
    return run(
        () -> {
          double speedMultiplier = level.get().maxSpeed;
          SmartDashboard.putNumber("limiter", speedMultiplier);
          double fieldCentricthrottle =
              (ControllerConstants.modifyAxisWithCustomDeadband(
                  fieldCentricthrottleSupplier.get(), 0.08, 1));
          double robotCentricThrottle =
              (ControllerConstants.modifyAxisWithCustomDeadband(
                      robotCentricthrottleSupplier.get(), 0.08, 2)
                  / 2);
          ControllerConstants.modifyAxis(xSupplier.get());
          ControllerConstants.modifyAxis(ySupplier.get());
          double rotation =
              ControllerConstants.modifyAxisWithCustomDeadband(rotationSupplier.get(), 0.06, 1) / 2;
          double x = ControllerConstants.modifyAxis(-xSupplier.get());
          double y = ControllerConstants.modifyAxis(-ySupplier.get());
          double activeThrottle;

          if (fieldCentricthrottle != 0) {
            activeThrottle = fieldCentricthrottle;
          } else {
            activeThrottle = robotCentricThrottle;
          }

          boolean isBraking = false;

          if (!(x == 0 && y == 0) && !(fieldCentricthrottleSupplier.get().intValue() == 0 && robotCentricthrottleSupplier.get().intValue() == 0)/* no gas */) {
            double angle = Math.atan2(x, y) + Math.PI / 2;
            x = Math.cos(angle) * activeThrottle;
            y = Math.sin(angle) * activeThrottle;
          } else if (x == 0 && y == 0 && rotation == 0) {
            // robot is not receiving input
            ChassisSpeeds speeds = getSpeeds();

            // are we near stop within a tolarance
            //if (MathUtil.isNear(0, speeds.vxMetersPerSecond, 0.01) && MathUtil.isNear(0, speeds.vyMetersPerSecond, 0.01) && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, 0.01)) {
              isBraking = true;
              brake();
            //}
          }

          if (!isBraking) {
            if (activeThrottle == robotCentricThrottle) {
              setControl(
                  m_RobotCentricdrive
                      .withVelocityX(-percentOutputToMetersPerSecond(m_xLimiter.calculate(x)))
                      .withDeadband(0.05)
                      .withVelocityY(percentOutputToMetersPerSecond(m_yLimiter.calculate(y)))
                      .withDeadband(0.05)
                      .withRotationalRate(
                          -percentOutputToRadiansPerSecond(m_rotationLimiter.calculate(rotation))));
            } else {
              setControl(
                  m_FieldCentricdrive
                      .withVelocityX(-percentOutputToMetersPerSecond(m_xLimiter.calculate(x)))
                      .withDeadband(0.05)
                      .withVelocityY(percentOutputToMetersPerSecond(m_yLimiter.calculate(y)))
                      .withDeadband(0.05)
                      .withRotationalRate(
                          -percentOutputToRadiansPerSecond(m_rotationLimiter.calculate(rotation))));
            }
          }

          SmartDashboard.putNumber("FieldCentricthrottle", fieldCentricthrottle);
          SmartDashboard.putNumber("RobotCentricThrottle", robotCentricThrottle);
          SmartDashboard.putBoolean("XBrake", isBraking);
        });
  }

  public double percentOutputToMetersPerSecond(double percentOutput) {
    return DrivetrainConstants.maxSpeedMetersPerSecond * percentOutput;
  }

  public double percentOutputToRadiansPerSecond(double percentOutput) {
    return DrivetrainConstants.maxAngularVelocityRadiansPerSecond * percentOutput;
  }

  public Command zero_pidgeon() {
    return runOnce(this::seedFieldCentric);
  }

  public Command goToPose(Pose2d pose) {
    double[] debugArray = {pose.getX(), pose.getY()};
    SmartDashboard.putNumberArray("Going to Pose", debugArray);

    return AutoBuilder.pathfindToPose(pose, DrivetrainConstants.pathConstraints);
  }

  public Command goToPose(Supplier<Pose2d> pose) {
    return goToPose(pose.get());
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  public Command cardinalMovement(double velocityX, double velocityY) {
    return run(
        () -> {
          setControl(
              m_RobotCentricdrive
                  .withVelocityY(percentOutputToMetersPerSecond(m_yLimiter.calculate(velocityY)))
                  .withVelocityX(percentOutputToMetersPerSecond(m_xLimiter.calculate(velocityX)))
                  .withRotationalRate(0.0));
        });
  }

  private void brake() {
    setControl(m_brake);
  }

  private void horizontalAdjust(Supplier<Double> horizontalError, double skew) {
    run(() -> horizontalDrive(horizontalError, skew));
  }

  private void horizontalDrive(Supplier<Double> horizontalError, double skew) {
    double xError = horizontalError.get() + skew;
    m_RobotCentricdrive
        .withVelocityX(m_xController.calculate(xError))
        .withVelocityY(0)
        .withRotationalRate(0);
  }

  private boolean xVisonPIDAtSetpoint() {
    return m_xController.atSetpoint();
  }

  private boolean yVisonPIDAtSetpoint() {
    return m_yController.atSetpoint();
  }

  private boolean visonPIDsAtSetpoint() {
    return xVisonPIDAtSetpoint() && yVisonPIDAtSetpoint();
  }

  /*
   * Adjusts the robot so that the supplied errors match the target erros from the april tag.
   */
  public Command visonAdjust(
      Supplier<Double> horizontalError, Supplier<Double> verticalError, Supplier<Double> horizontalTarget, Supplier<Double> verticalTarget, Supplier<Integer> inversionSupplier) {
    return run(
        () -> {
          SmartDashboard.putNumber("Vertical Error", verticalError.get());
          SmartDashboard.putNumber("Horizontal Error", horizontalError.get());
          SmartDashboard.putBoolean("Running Limelight", true);
          setControl(
            m_RobotCentricdrive
               //.withVelocityX(inversionSupplier.get()*m_xController.calculate(verticalError.get(), verticalTarget.get()))
                .withVelocityY(inversionSupplier.get()*m_yController.calculate(horizontalError.get(), horizontalTarget.get()))
                .withVelocityX(0.01)
                .withRotationalRate(0));
        }).until(this::visonPIDsAtSetpoint);
  }

  public Command visonAdjustVertical(
    Supplier<Double> verticalError, Supplier<Double> verticalTarget, Supplier<Integer> inversionSupplier) {
      return run(
          () -> {
            SmartDashboard.putNumber("Vertical Error", verticalError.get());
            SmartDashboard.putBoolean("Running Limelight", true);
            setControl(
              m_RobotCentricdrive
                  .withVelocityY(0.0)
                  .withVelocityX(inversionSupplier.get()*m_xController.calculate(verticalError.get(), verticalTarget.get()))
                  .withRotationalRate(0));
          }).until(this::xVisonPIDAtSetpoint).withTimeout(1.0);
  }

  public Command visonAdjustHorrizontal(
    Supplier<Double> horizontalError, Supplier<Double> horizontalTarget, Supplier<Integer> inversionSupplier) {
      return run(
          () -> {
            SmartDashboard.putNumber("Horizontal Error", horizontalError.get());
            SmartDashboard.putBoolean("Running Limelight", true);
            setControl(
              m_RobotCentricdrive
                  .withVelocityX(0.0)
                  .withVelocityY(inversionSupplier.get()*m_yController.calculate(horizontalError.get(), horizontalTarget.get()))
                  .withRotationalRate(0));
          }).until(this::yVisonPIDAtSetpoint).withTimeout(1.0);
  }

  public Command visonAdjustTimeout(
      Supplier<Double> horizontalError, Supplier<Double> verticalError, Supplier<Double> horizontalTarget, Supplier<Double> verticalTarget, Supplier<Integer> inversionSupplier) {
    return run(
        () -> {
          SmartDashboard.putNumber("Vertical Error", verticalError.get());
          SmartDashboard.putNumber("Horizontal Error", horizontalError.get());
          SmartDashboard.putBoolean("Running Limelight", true);
          setControl(
            m_RobotCentricdrive
                .withVelocityX(inversionSupplier.get()*m_xController.calculate(verticalError.get(), verticalTarget.get()))
                .withVelocityY(inversionSupplier.get()*m_yController.calculate(horizontalError.get(), horizontalTarget.get()))
                .withRotationalRate(0));
        }).withTimeout(Seconds.of(0.75)).finallyDo(()->{System.out.println("Finsihd Liemligth Cmomand");}).handleInterrupt(()->{System.out.println("Inturrpted Limeliught Commamd");});
  }

  public Command setPose(Pose2d pose){
    return runOnce(
      ()->{
        if (DriverStation.getAlliance().isPresent()){
          if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            resetPose(FlippingUtil.flipFieldPose(pose));
          }}
        else{
          resetPose(pose);
        }
      });
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /** Sets Supply Current Limits for the drive motors based on the values in DrivetrainConstants. */
  private void applySupplyCurrentLimits() {
    // Set current limits for the drive motors
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
      TalonFXConfigurator configurator = module.getDriveMotor().getConfigurator();
      TalonFXConfiguration config = new TalonFXConfiguration();
      // Get the current limits from the constants to avoid wipe out of existing values we don't set
      module.getDriveMotor().getConfigurator().refresh(config);
      CurrentLimitsConfigs currentLimitConfigs = config.CurrentLimits;
      currentLimitConfigs
          .withSupplyCurrentLowerLimit(CurrentLimits.kDriveCurrentLimitMin)
          .withSupplyCurrentLimit(CurrentLimits.kDriveCurrentLimitMax)
          .withSupplyCurrentLowerTime(CurrentLimits.kDriveCurrentDuration)
          .withSupplyCurrentLimitEnable(true);
      configurator.apply(config);
    }
  }

  private void configureAutobuilder() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose2d, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting
          // pose)
          this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(
                          feedforwards
                              .robotRelativeForcesYNewtons())), // Method that will drive the robot
          // given ROBOT RELATIVE
          // ChassisSpeeds. Also optionally
          // outputs individual module
          // feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
              // controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  private ChassisSpeeds getSpeeds() {
    return getState().Speeds;
  }

  private Rotation2d getRotation2d() {
    return getState().RawHeading;
  }
}
