package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;
import frc.robot.util.PhoenixUtil;

public class DriveSubsystem extends SubsystemBase {

    // Center motors are leaders — physically centered in each gear train
    private final TalonFX m_leftLeader  = new TalonFX(Drivetrain.CENTER_LEFT_MOTOR_ID);
    private final TalonFX m_rightLeader = new TalonFX(Drivetrain.CENTER_RIGHT_MOTOR_ID);

    // Front and back motors follow their side's center leader
    private final TalonFX m_frontLeft  = new TalonFX(Drivetrain.FRONT_LEFT_MOTOR_ID);
    private final TalonFX m_backLeft   = new TalonFX(Drivetrain.BACK_LEFT_MOTOR_ID);
    private final TalonFX m_frontRight = new TalonFX(Drivetrain.FRONT_RIGHT_MOTOR_ID);
    private final TalonFX m_backRight  = new TalonFX(Drivetrain.BACK_RIGHT_MOTOR_ID);

    // UpdateFreqHz = 0 so requests are sent immediately on each setControl() call
    private final DutyCycleOut m_leftOut  = new DutyCycleOut(0);
    private final DutyCycleOut m_rightOut = new DutyCycleOut(0);

    // Rate-limit throttle only; turning stays responsive for parade maneuvering
    private final SlewRateLimiter m_throttleLimiter = new SlewRateLimiter(Drivetrain.SLEW_RATE);

    public DriveSubsystem() {
        // Configure all left motors CCW+, all right motors CW+
        PhoenixUtil.configureDriveMotor(m_leftLeader.getConfigurator(),  InvertedValue.CounterClockwise_Positive);
        PhoenixUtil.configureDriveMotor(m_frontLeft.getConfigurator(),   InvertedValue.CounterClockwise_Positive);
        PhoenixUtil.configureDriveMotor(m_backLeft.getConfigurator(),    InvertedValue.CounterClockwise_Positive);

        PhoenixUtil.configureDriveMotor(m_rightLeader.getConfigurator(), InvertedValue.Clockwise_Positive);
        PhoenixUtil.configureDriveMotor(m_frontRight.getConfigurator(),  InvertedValue.Clockwise_Positive);
        PhoenixUtil.configureDriveMotor(m_backRight.getConfigurator(),   InvertedValue.Clockwise_Positive);

        // Wire followers to their leader — Aligned = same output direction as leader
        m_frontLeft.setControl(new Follower(m_leftLeader.getDeviceID(),  MotorAlignmentValue.Aligned));
        m_backLeft.setControl( new Follower(m_leftLeader.getDeviceID(),  MotorAlignmentValue.Aligned));
        m_frontRight.setControl(new Follower(m_rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));
        m_backRight.setControl( new Follower(m_rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));

        m_leftOut.UpdateFreqHz  = 0;
        m_rightOut.UpdateFreqHz = 0;
    }

    /**
     * Arcade drive. Call every loop (e.g., as a default command).
     *
     * @param throttle Forward/reverse [-1, 1]; positive = forward. Apply deadband before calling.
     * @param turn     Left/right [-1, 1]; positive = turn right. Apply deadband before calling.
     */
    public void arcadeDrive(double throttle, double turn) {
        double limitedThrottle = m_throttleLimiter.calculate(throttle * Drivetrain.MAX_OUTPUT);
        double scaledTurn = turn * Drivetrain.MAX_OUTPUT;

        m_leftOut.Output  = limitedThrottle + scaledTurn;
        m_rightOut.Output = limitedThrottle - scaledTurn;

        m_leftLeader.setControl(m_leftOut);
        m_rightLeader.setControl(m_rightOut);
    }
}
