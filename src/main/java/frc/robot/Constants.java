// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static class SwerveConstants{
        public static int frontLeftDriveMotorID = 13;      // FX
        public static int frontLeftTurnMotorID = 12;        // SRX

        public static int frontRightDriveMotorID = 02;
        public static int frontRightTurnMotorID = 03;

        public static int backLeftDriveMotorID = 14;
        public static int backLeftTurnMotorID = 15;

        public static int backRightDriveMotorID = 01;
        public static int backRightTurnMotorID = 00;

    }

    public static class ControllerConstants{
        public static int turnStrafeStickID = 2;
        public static int rotateStickID = 1;
    }
}
