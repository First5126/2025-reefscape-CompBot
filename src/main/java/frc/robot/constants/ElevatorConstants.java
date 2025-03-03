package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Supplier;

public class ElevatorConstants {
  public static final int ELEVATOR_MOTOR_PORT = 1;
  public static final double ELEVATOR_SPEED = 0.75;
  public static final double ELEVATOR_MAX_HEIGHT = 2.0; // in meters
  public static final double ELEVATOR_MIN_HEIGHT = 0.0; // in meters
  public static final double ELEVATOR_TOLERANCE = 0.01; // in meters

  public static final double kP = 15;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kV = 0;
  public static final double kS = 0.16;
  public static final double kG = 0.032;

  public static final int FORWARD_DIGITAL_LIMIT = 0;
  public static final int REVERSE_DIGITAL_LIMIT = 0;

  public static final double GEAR_RATIO = 12.0;

  public static final Angle ELEVATOR_READING_STDV = Revolutions.of(0.05);

  public static final Supplier<Double> TRIM_UP_SPEED = () -> Double.valueOf(1);
  public static final Supplier<Double> TRIM_DOWN_SPEED = () -> Double.valueOf(-1);

  public static class ElevatorLevel {
    public final Angle height;

    private ElevatorLevel(double height) {
      this.height = Revolutions.of(height);
    }

    public static final ElevatorLevel CORAL_L1 = new ElevatorLevel(0d);
    public static final ElevatorLevel CORAL_L2 = new ElevatorLevel(1.3);
    public static final ElevatorLevel CORAL_L3 = new ElevatorLevel(2.116);
    public static final ElevatorLevel CORAL_L4 = new ElevatorLevel(4.99);
    public static final ElevatorLevel CORAL_STATION = new ElevatorLevel(0.914795);
    public static final ElevatorLevel TRAVEL = new ElevatorLevel(0.5);
    public static final ElevatorLevel ALGAE_L2 = new ElevatorLevel(2.8);
    public static final ElevatorLevel ALGAE_L3 = new ElevatorLevel(4.17);
    public static final ElevatorLevel PROCESSER = new ElevatorLevel(0.722);
  }

  public static class BaseLevel {
    public final ElevatorLevel level;

    private BaseLevel(ElevatorLevel level) {
      this.level = level;
    }
  }

  public static class IntakeOuttake extends BaseLevel {
    public final Voltage intakeVoltage;
    public final Voltage outakeVoltage;
    public final Angle angle;

    private IntakeOuttake(ElevatorLevel level, double intake, double outtake, Angle angle) {
      super(level);
      this.intakeVoltage = Volts.of(intake);
      this.outakeVoltage = Volts.of(outtake);
      this.angle = angle;
    }
  }

  public static class AlgaeLevel extends IntakeOuttake {
    private AlgaeLevel(ElevatorLevel level, double intake, double outtake, double angle) {
      super(level, intake, outtake, Revolutions.of(angle));
    }

    public static final AlgaeLevel L2 = new AlgaeLevel(ElevatorLevel.ALGAE_L2, 0.21, 0.0, 5d);
    public static final AlgaeLevel L3 = new AlgaeLevel(ElevatorLevel.ALGAE_L3, 0.21, 0.0, 5d);
    public static final AlgaeLevel PROCESSOR =
        new AlgaeLevel(ElevatorLevel.PROCESSER, 0.21, 0.0, 0d);
  }

  public static class CoralLevel extends IntakeOuttake {
    private CoralLevel(ElevatorLevel level, double intake, double outtake, double angle) {
      super(level, intake, outtake, Degrees.of(angle));
    }

    public static final CoralLevel L1 = new CoralLevel(ElevatorLevel.L1, -8d, 8d, 5d);
    public static final CoralLevel L2 = new CoralLevel(ElevatorLevel.L2, -8d, 8d, 3.5);
    public static final CoralLevel L3 = new CoralLevel(ElevatorLevel.L3, -8d, 8d, 3.5);
    public static final CoralLevel L4 = new CoralLevel(ElevatorLevel.L4, -8d, 8d, 3.5);
    public static final CoralLevel STATION =
        new CoralLevel(ElevatorLevel.CORAL_STATION, -5d, 5d, 74.5);
  }
}
