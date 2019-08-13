package org.minutebots.frc2019;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Robot extends TimedRobot {

  private Joystick joystick = new Joystick(0);
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private JoystickButton resetGyro = new JoystickButton(joystick, 10);
  private JoystickButton yeetForwards = new JoystickButton(joystick,7),
          yeetBackwards = new JoystickButton(joystick,9);
  private JoystickButton grabHatch = new JoystickButton(joystick,3),
          ejectHatch = new JoystickButton(joystick,5);
  private JoystickButton lockBall = new JoystickButton(joystick,4),
          ejectBall = new JoystickButton(joystick,6);

  private Runnable driveTrain = new Runnable() { //TODO: Add vision functionality
    private WPI_VictorSPX leftFront = new WPI_VictorSPX(0),
            rightFront = new WPI_VictorSPX(1),
            leftBack = new WPI_VictorSPX(3),
            rightBack = new WPI_VictorSPX(2);
    private MecanumDrive mecanumDrive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
    public void run() {
      if(resetGyro.get()) gyro.reset();
      mecanumDrive.driveCartesian(joystick.getX(), -joystick.getY(), joystick.getTrigger() ? 0.6*joystick.getTwist():0, gyro.getAngle());
    }
  };

  private Runnable depotArm = new Runnable() {
    private boolean isDown = false;

    private SpeedController wheelMotor = new VictorSP(0),
            armMotor = new Spark(2);

    private DigitalInput upLimit = new DigitalInput(0), //These are reversed
            downLimit = new DigitalInput(1);

    private void moveArm(double speed) {
      double motorSpeed = speed;
      if (upLimit.get()) motorSpeed = speed < 0 ? speed : 0;
      if (downLimit.get()) motorSpeed = speed > 0 ? speed : 0;
      armMotor.set(motorSpeed);
    }

    private void spinWheel(double speed) {
      wheelMotor.set(speed);
    }

    public void run() {
      if (isDown) moveArm(-0.2); //depot arm goes down
      else moveArm(0.2); //black hawk going up
      if(yeetForwards.get() || yeetBackwards.get()){
        isDown = true;
        spinWheel((yeetForwards.get() ? 1 : -1) * -1.0);
      } else {
        isDown = false;
        spinWheel(0);
      }
    }
  };

  private Runnable hatchMech = new Runnable() {
    private DoubleSolenoid piston = new DoubleSolenoid(6, 7),
            activeHatch = new DoubleSolenoid(4, 5);

    public void run() {
      if(grabHatch.get()) activeHatch.set(DoubleSolenoid.Value.kForward);
      else activeHatch.set(DoubleSolenoid.Value.kReverse);
      if(ejectHatch.get()) piston.set(DoubleSolenoid.Value.kForward);
      else piston.set(DoubleSolenoid.Value.kReverse);
    }
  };

  private Runnable ramp = new Runnable() {
    private VictorSP rampMotor = new VictorSP(2);
    private DoubleSolenoid rampLock = new DoubleSolenoid(3, 2);

    public void run() {
      if(lockBall.get()) rampLock.set(DoubleSolenoid.Value.kForward);
      if(ejectBall.get()){
        rampLock.set(DoubleSolenoid.Value.kReverse);
        rampMotor.set(0.3);
      } else rampMotor.set(0.0);
    }
  };

  public void teleopPeriodic() {
    driveTrain.run();
    depotArm.run();
    hatchMech.run();
    ramp.run();
  }
}
