package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.influxdb.InfluxDB;
import org.influxdb.InfluxDBFactory;
import org.influxdb.dto.BatchPoints;
import org.influxdb.dto.Point;
import org.influxdb.dto.Pong;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class InfluxDbLogger {

    String serverURL = "http://192.168.43.231:8086";
    String username = "admin";
    String password = "admin123";
    String databaseName = "ftc";

    InfluxDB influxDB;

    HardwareMap hardwareMap;

    Map<String, Object> fields = new HashMap<>();

    boolean isConnected = false;

    public InfluxDbLogger(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;
        influxDB = InfluxDBFactory.connect(serverURL, username, password); // (1)
        try
        {
            Pong response = this.influxDB.ping();
            if (!response.getVersion().equalsIgnoreCase("unknown"))
            {
                influxDB.createDatabase(databaseName);
                isConnected = true;
            }
        } catch (Exception ignored)
        {

        }
    }
    public boolean isConnected() {
        return isConnected;
    }

    public void add(String field, Object value)
    {
        fields.put(field, value);
    }

    public void write()
    {
        if (!isConnected)
        {
            return;
        }
        Battery battery = new Battery(hardwareMap);
        IMU imu = new IMU(hardwareMap);
        Point point = Point.measurement("robot") // (6)
                .time(System.currentTimeMillis(), TimeUnit.MILLISECONDS).fields(fields)
                .fields(imu.getAttitude()).addField("battery_voltage", battery.getVoltage())
                .build();

        BatchPoints batchPoints = BatchPoints.database(databaseName).build();
        batchPoints.point(point);
        influxDB.write(batchPoints);
        fields.clear();
    }}
