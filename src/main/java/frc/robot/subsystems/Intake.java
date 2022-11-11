// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CTREConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LoaderConstants;

public class Intake extends SubsystemBase {

  public final WPI_TalonFX m_intake = new WPI_TalonFX(IntakeConstants.kIntakeMotorCanId, CTREConstants.kCanivoreName);
  public final WPI_TalonFX m_mover = new WPI_TalonFX(IntakeConstants.kMoverMotorCanId, CTREConstants.kCanivoreName);
  public final WPI_TalonFX m_shooterLoader = new WPI_TalonFX(LoaderConstants.kLoaderMotorCanId, CTREConstants.kCanivoreName);

  public final DoubleSolenoid m_doublePCM = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      IntakeConstants.kIntakePneumatic1, IntakeConstants.kIntakePneumatic2);

  private final ColorSensor m_colorSensor = new ColorSensor();

  private final DigitalInput m_middleDIO = new DigitalInput(IntakeConstants.kIntakeMiddleDio);
  private final DigitalInput m_loadDIO = new DigitalInput(LoaderConstants.kLoaderDio);

  private Timer m_timer = new Timer();

  ShuffleboardTab debugTab = Shuffleboard.getTab("computil");

  NetworkTableEntry ntColorFilterEnable = debugTab.add("Color Filter Enable", true)
      .withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 1).getEntry();

  private final double kIntakeOut = 0.9;
  private final double kBTOut = 0.8;
  private final double kLoaderOut = 0.5;
  private final double kLoaderLoadingSpeed = 0.8;

  private boolean isChanging = false;

  enum STATE {
    A, B, C, D, E, F, G;
  };

  STATE m_state = STATE.A;

  /** Creates a new Intake. */
  public Intake() {
    m_colorSensor.init();

    m_intake.configFactoryDefault(100);
    m_mover.configFactoryDefault(100);
    m_shooterLoader.configFactoryDefault(100);

    m_intake.setInverted(true);
    m_mover.setInverted(false);
    m_shooterLoader.setInverted(false);


    m_intake.setNeutralMode(NeutralMode.Brake);
    m_mover.setNeutralMode(NeutralMode.Brake);
    m_shooterLoader.setNeutralMode(NeutralMode.Brake);


    // add to shuffleboard
    debugTab.addBoolean("Red Detect", () -> m_colorSensor.isRedBallDetected());
    debugTab.addBoolean("Blue Detect", () -> m_colorSensor.isBlueBallDetected());

    m_doublePCM.set(Value.kReverse);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_colorSensor.checkColor();
  }

  public void turnOn() {
    if (getNumberOfBallsHolding() < 2) {
      m_state = STATE.B;
    }
  }

  public void changeState(boolean startIntake, boolean stopIntake, boolean reverse) {
    switch (m_state) {
      case A:

        // intake and load off, intake up

        if (startIntake) {
          if (isBallAtMiddle() && isBallAtLoad()) {
            m_state = STATE.A;
          } else {
            m_state = STATE.B;
          }
        }

        if (stopIntake) {
          // don't care
        }

        if (reverse) {
          m_state = STATE.G;
        }

        if (isBallAtLoad() || isBallAtMiddle()) {
          // don't care
        }

        // move movers if ball not at load shooter place
        if (isBallAtMiddle() && !isBallAtLoad()) {
          m_state = STATE.F;
        }

        break;
      case B:
        // intake, bt, and load ON
        // intake down
        if (startIntake) {
          // don't care
        }

        if (stopIntake) {
          m_state = STATE.A;
        }

        if (reverse) {
          m_state = STATE.G;
        }

        // ball detected right before shooter, go to next state
        if (isBallAtLoad()) {
          m_state = STATE.C;
        }

        // SPIT BALL OUT IF BAD (WRONG COLOR) :)))))
        spitWrongColorBallOut();

        break;
      case C:
        // intake, mover, ON, loader OFF
        // intake down

        // inputs
        if (startIntake) {
          // don't care
        }
        if (stopIntake) {
          m_state = STATE.A;
        }
        if (reverse) {
          m_state = STATE.G;
        }
        if (isBallAtLoad()) {
          // don't care, ball already there
        } else {
          m_state = STATE.B;
        }
        if (isBallAtMiddle() && isBallAtLoad()) {
          // 2nd ball loaded, stop intaking
          m_state = STATE.A;
        }

        // SPIT BALL OUT IF BAD (WRONG COLOR) :))))))
        spitWrongColorBallOut();

        break;
      case D:
        // state changes to E after timer (inbetween state)
        // m_timer.schedule(m_changeToE, 200);

        // m_intake.set(ControlMode.PercentOutput, kDefaultMotorSpeed);
        // m_mover.set(ControlMode.PercentOutput, kDefaultMotorSpeed);
        // m_shooterLoader.set(ControlMode.PercentOutput, 0);

        // m_doublePCM.set(Value.kForward);

        break;
      case E:
        // start loading balls into shooter (loadShooter)
        // stops when no more shooter buttons are pressed

        if (startIntake) {
          m_state = STATE.B;
        }
        if (stopIntake) {
          m_state = STATE.A;
        }

        break;
      case F:
        // move ball at color sensor location to shooter loader
        // intake up

        if (startIntake) {
          m_state = STATE.B;
        }

        if (stopIntake) {
          m_state = STATE.A;
        }

        if (reverse) {
          m_state = STATE.G;
        }

        // ball detected right before shooter, go to next state
        if (isBallAtLoad()) {
          m_state = STATE.A;
        }

        break;
      case G:
        if (startIntake) {
          m_state = STATE.B;
        }

        if (stopIntake) {
          m_state = STATE.A;
        }
        if (reverse) {
          // already here
        }

        break;
    }

    // actual motor states
    switch (m_state) {
      case A:
        // intake and load off, intake up

        m_intake.set(ControlMode.PercentOutput, 0);
        m_mover.set(ControlMode.PercentOutput, 0);
        m_shooterLoader.set(ControlMode.PercentOutput, 0);

        setIntakePnuematic(false);

        break;
      case B:
        // intake, bt, and load ON
        // intake down
        if (!isChanging) {
          m_intake.set(ControlMode.PercentOutput, kIntakeOut);
          m_mover.set(ControlMode.PercentOutput, kBTOut);
        } else {
          reverseIntake();
          if (m_timer.get() >= .4) {
            isChanging = false;
            m_timer.stop();
          }
        }

        m_shooterLoader.set(ControlMode.PercentOutput, kLoaderOut);

        // m_doublePCM.set(Value.kForward);
        setIntakePnuematic(true);

        break;
      case C:
        // intake, mover, ON, loader OFF
        // intake down

        if (!isChanging) {
          m_intake.set(ControlMode.PercentOutput, kIntakeOut);
          m_mover.set(ControlMode.PercentOutput, kBTOut);
        } else {
          reverseIntake();
          if (m_timer.get() >= .4) {
            isChanging = false;
            m_timer.stop();
          }
        }

        m_shooterLoader.set(ControlMode.PercentOutput, 0);

        setIntakePnuematic(true);

        // m_doublePCM.set(Value.kForward);
        break;
      case D:
        // state changes to E after timer (inbetween state)
        // m_timer.schedule(m_changeToE, 200);

        // m_intake.set(ControlMode.PercentOutput, kDefaultMotorSpeed);
        // m_mover.set(ControlMode.PercentOutput, kDefaultMotorSpeed);
        // m_shooterLoader.set(ControlMode.PercentOutput, 0);

        // m_doublePCM.set(Value.kForward);
        break;
      case E:
        // start loading balls into shooter (loadShooter)
        // stops when no more shooter buttons are pressed
        m_intake.set(ControlMode.PercentOutput, 0.0);
        m_mover.set(ControlMode.PercentOutput, kBTOut);
        m_shooterLoader.set(ControlMode.PercentOutput, kLoaderLoadingSpeed);
        break;
      case F:
        // mover, ON,intake, loader OFF
        // intake up

        m_intake.set(ControlMode.PercentOutput, 0);
        m_mover.set(ControlMode.PercentOutput, kBTOut);

        m_shooterLoader.set(ControlMode.PercentOutput, kLoaderOut);

        setIntakePnuematic(false);
        break;
      case G:
        // reverse
        m_intake.set(ControlMode.PercentOutput, -0.75);
        m_mover.set(ControlMode.PercentOutput, -0.5);
        m_shooterLoader.set(ControlMode.PercentOutput, 0);
        setIntakePnuematic(true);
        break;
    }

    // sets nt values for motors
    // setNTValues();

  }

  // toggles intake pnuematic, for testing purposes
  public void setIntakePnuematic(boolean forward) {
    m_doublePCM.set(forward ? Value.kForward : Value.kReverse);
  }

  // Turns on shooter intake and mover to put balls in shooter.
  // Used in OI to coordinate shooting.
  public void loadShooter() {
    m_state = STATE.E;
  }

  // Goes back to the first state
  public void turnOffLoadShooter() {
    m_state = STATE.A;
  }

  public void stopMotors() {
    m_state = STATE.A;
    m_intake.stopMotor();
    m_mover.stopMotor();
    m_shooterLoader.stopMotor();
  }

  // continues spinning intake motors if ball is there while shooting
  public void countinueIntakeMotors() {
    if (isBallAtLoad()) {
      m_state = STATE.A;
    } else {
      m_state = STATE.F;
    }
  }

  // spit out ball
  public void reverseIntake() {
    m_mover.set(ControlMode.PercentOutput, -0.45);
    m_intake.set(ControlMode.PercentOutput, -0.45);
  }

  public boolean unjamShooter() {
    m_mover.set(ControlMode.PercentOutput, -0.2);
    m_shooterLoader.set(ControlMode.PercentOutput, -0.2);
    return isBallAtLoad();
  }

  // sends motor values to shuffleboard
  public void setNTValues() {
    // ntIntakeSpeed.setDouble(m_intake.getMotorOutputPercent());
    // ntMoverSpeed.setDouble(m_mover.getMotorOutputPercent());
    // ntShooterLoaderSpeed.setDouble(m_shooterLoader.getMotorOutputPercent());
  }

  public boolean getBothBallsLoaded() {
    if (isBallAtLoad() && isBallAtMiddle()) {
      return true;
    }

    return false;
  }

  // helper functions so don't have to remember to invert DIO
  public boolean isBallAtLoad() {
    return !m_loadDIO.get();
  }

  public boolean isBallAtMiddle() {
    return !m_middleDIO.get();
  }

  public int getNumberOfBallsHolding() {
    int count = 0;
    count += isBallAtLoad() ? 1 : 0;
    count += (isBallAtMiddle()) ? 1 : 0;
    return count;
  }

  private void spitWrongColorBallOut() {
    switch (ntColorFilterEnable.getBoolean(true) ? DriverStation.getAlliance() : Alliance.Invalid) {
      case Blue:
        if (m_colorSensor.isRedBallDetected()) {
          // turns back to normal (isChanging = False) after 1 seconds
          startSpitOutTimer();
        }
        break;
      case Red:
        if (m_colorSensor.isBlueBallDetected()) {
          // turns back to normal (isChanging = False) after 1 seconds
          startSpitOutTimer();
        }
        break;
      case Invalid:
        break;
    }
  }

  private void startSpitOutTimer() {
    isChanging = true;

    m_timer.reset();
    m_timer.start();
  }
}
