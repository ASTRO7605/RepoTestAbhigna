// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//ajout pour limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
//
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    private final SparkMax m_leftDriveAV = new SparkMax(1, MotorType.kBrushed);
    private final SparkMax m_rightDriveAV = new SparkMax(3, MotorType.kBrushed);
    private final SparkMax m_leftDriveARslave = new SparkMax(2, MotorType.kBrushed);
    private final SparkMax m_rightDriveARslave = new SparkMax(4, MotorType.kBrushed);

    // ajout pour limelight (initialisation du limelight 4) (utile pour avoir les
    // données du limelight)
    private final NetworkTable limelight_m = NetworkTableInstance.getDefault().getTable("limelight");
    //

    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDriveAV::set, m_rightDriveAV::set);
    private final XboxController m_controller = new XboxController(0);
    private final Timer m_timer = new Timer();

    public Robot() {
        var leftConfig = new SparkMaxConfig();
        leftConfig.follow(m_leftDriveAV, false);
        m_leftDriveARslave.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var rightConfig = new SparkMaxConfig();
        rightConfig.follow(m_rightDriveAV, false);
        m_rightDriveARslave.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SendableRegistry.addChild(m_robotDrive, m_leftDriveAV);
        SendableRegistry.addChild(m_robotDrive, m_rightDriveAV);
        SendableRegistry.addChild(m_robotDrive, m_leftDriveARslave);
        SendableRegistry.addChild(m_robotDrive, m_rightDriveARslave);
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        var invertedconfig = new SparkMaxConfig();
        invertedconfig.inverted(false);
        m_rightDriveAV.configure(invertedconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        m_timer.restart();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // Drive for 2 seconds
        if (m_timer.get() < 2.0) {
            // Drive forwards half speed, make sure to turn input squaring off
            m_robotDrive.arcadeDrive(0.5, 0.0, false);
        } else {
            m_robotDrive.stopMotor(); // stop robot
        }

        // Exercises - faire avancer le robot à partir des données du limelight
        // si le robot est assez proche le moteur arrête, sinon il continue à avancer.
        ///////////////////////////////////////////////////////////////////
        double tv = limelight_m.getEntry("tv").getDouble(0.0);
        double ta = limelight_m.getEntry("ta").getDouble(0.0);

        if (tv < 1.0) {
            m_robotDrive.stopMotor();
            return;
        }

        double STOP_AREA = 1.5;

        if (ta >= STOP_AREA) {
            m_robotDrive.stopMotor();
        } else {
            m_robotDrive.arcadeDrive(0.5, 0.0);
        }
        /// //////////////////////////////////////////////////////////////

    }

    /**
     * This function is called once each time the robot enters teleoperated mode.
     */
    @Override
    public void teleopInit() {
    }

    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic() {
        m_robotDrive.arcadeDrive(-m_controller.getLeftY() * 0.9, -m_controller.getRightX() * 0.7);
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
