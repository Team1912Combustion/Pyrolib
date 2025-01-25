package org.team1912.pyrogen.pyrolib.OTOS;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;

@I2cDeviceType
@DeviceProperties(
        name = "Extend SparkFun OTOS",
        xmlTag = "OTOSSensor",
        description = "Extend SparkFun Qwiic Optical Tracking Odometry Sensor"
)
public class OTOSSensor extends SparkFunOTOS {
    public OTOSSensor(I2cDeviceSynch deviceClient)
    {
        super(deviceClient);
    }

    public Pose2d getPose2d() {
        Pose2D sensor_pos = getPosition();
        return new Pose2d(
                -1. * sensor_pos.y,
                sensor_pos.x,
                Rotation2d.fromDegrees(sensor_pos.h)
        );
    }

    public void setPose2d(Pose2d pose2d) {
        Pose2D sensor_pos = new Pose2D();
        sensor_pos.x = pose2d.getY();
        sensor_pos.y = -1. * pose2d.getX();
        sensor_pos.h = pose2d.getHeading();
        this.setPosition(sensor_pos);
    }

}
