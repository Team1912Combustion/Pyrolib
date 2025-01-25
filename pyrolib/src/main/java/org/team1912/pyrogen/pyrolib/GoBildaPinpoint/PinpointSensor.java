package org.team1912.pyrogen.pyrolib.GoBildaPinpoint;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;

@I2cDeviceType
@DeviceProperties(
        name = "Pyrogen goBILDA Pinpoint",
        xmlTag = "PyroPinpoint",
        description = "Pyrogen extended goBILDA Pinpoint Odometry Device"
)
public class PinpointSensor extends GoBildaPinpointDriver {
    public PinpointSensor(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);
    }

    public Pose2d getPose2d() {
        Pose2D sensor_pos = getPosition();
        return new Pose2d(
                sensor_pos.getX(DistanceUnit.INCH),
                sensor_pos.getY(DistanceUnit.INCH),
                Rotation2d.fromDegrees(sensor_pos.getHeading(AngleUnit.DEGREES))
        );
    }

    public void setPose2d(Pose2d pose2d) {
        Pose2D pose2D = new Pose2D(DistanceUnit.INCH,
                pose2d.getX(),
                pose2d.getY(),
                AngleUnit.RADIANS,
                pose2d.getHeading());
        this.setPosition(pose2D);
    }

}
