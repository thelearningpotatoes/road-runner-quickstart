package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.HashMap;
import java.util.Map;

public class IMU {
    com.qualcomm.robotcore.hardware.IMU imu;

    public IMU(HardwareMap hardwareMap)
    {
        imu = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");

        /*
         * The next two lines define Hub orientation. The Default Orientation (shown) is when a hub
         * is mounted horizontally with the printed logo pointing UP and the USB port pointing
         * FORWARD.
         *
         * To Do: EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will
        // cause a code exception.
        imu.initialize(new com.qualcomm.robotcore.hardware.IMU.Parameters(orientationOnRobot));
    }

    public com.qualcomm.robotcore.hardware.IMU getIMU() {
        return imu;
    }

    public Map<String, Object> getAttitude()
    {
        Map<String, Object> fields = new HashMap<>();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        fields.put("yaw", orientation.getYaw(AngleUnit.DEGREES));
        fields.put("pitch", orientation.getPitch(AngleUnit.DEGREES));
        fields.put("roll", orientation.getRoll(AngleUnit.DEGREES));
        fields.put("yaw_velocity", angularVelocity.zRotationRate);
        fields.put("pitch_velocity", angularVelocity.xRotationRate);
        fields.put("roll_velocity", angularVelocity.yRotationRate);

        return fields;
    }
}
