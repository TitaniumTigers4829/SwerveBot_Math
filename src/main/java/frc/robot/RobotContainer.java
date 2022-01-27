// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class RobotContainer {
    public final Drivetrain m_drivetrain;
    private Joystick turnStrafeStick;
    private Joystick rotateStick;

    public RobotContainer(){
        m_drivetrain = new Drivetrain();
        turnStrafeStick = new Joystick(ControllerConstants.turnStrafeStickID);
        rotateStick = new Joystick(ControllerConstants.rotateStickID);

        // m_drivetrain.setDefaultCommand(new SwerveDrive(m_drivetrain, ()->controller1.getRawAxis(0), ()->controller1.getRawAxis(1), ()->controller1.getRawAxis(2)));
        // m_drivetrain.setDefaultCommand(new SwerveDrive(m_drivetrain));
        // m_drivetrain.setDefaultCommand(new SRXTest(m_drivetrain));
        m_drivetrain.setDefaultCommand(new SwerveDrive(m_drivetrain, ()->turnStrafeStick.getX(), ()->turnStrafeStick.getY(), ()->rotateStick.getX()));

        configureButtonBindings();
    }

    public void configureButtonBindings(){
        System.out.println("Buttons configured");
    }
}
