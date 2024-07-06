package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Battery {
    final static String NAME = "10-cell, 12V Slim Battery";
    final static String SKU = "REV-31-1302";
    final static String VOLTAGE = "12V";
    final static String CAPACITY = "3000mAh";
    final static String WEIGHT = "567g";
    final static String WIRE_GAUGE = "16 AWG";
    final static String CONNECTOR = "XT30";

    HardwareMap hardwareMap;

    public Battery(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;
    }

    public double getVoltage()
    {
        double result = 0;
        // More than one voltage sensors, returning the max value
        for (VoltageSensor sensor : hardwareMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();
            if (voltage > 0)
            {
                result = Math.max(result, voltage);
            }
        }
        return result;
    }
}
