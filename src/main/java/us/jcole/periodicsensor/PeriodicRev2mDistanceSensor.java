package us.jcole.periodicsensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

class PeriodicRev2mDistanceSensor extends PeriodicSensor {
    private Rev2mDistanceSensor mSensor;

    PeriodicRev2mDistanceSensor(HardwareMap hardwareMap, String deviceName, int collectionPeriod) {
        super(hardwareMap, deviceName, collectionPeriod);
    }

    @Override
    protected void initializeSensor(final HardwareMap hardwareMap, final String deviceName) {
        mSensor = hardwareMap.get(Rev2mDistanceSensor.class, deviceName);
    }

    @Override
    protected double getSensorValue() {
        return mSensor.getDistance(DistanceUnit.MM);
    }
}
