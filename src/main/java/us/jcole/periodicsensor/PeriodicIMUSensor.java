package us.jcole.periodicsensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

class PeriodicIMUSensor extends PeriodicSensor {
    private BNO055IMU mSensor;

    PeriodicIMUSensor(HardwareMap hardwareMap, String deviceName, int collectionPeriod) {
        super(hardwareMap, deviceName, collectionPeriod);
    }

    @Override
    protected void initializeSensor(HardwareMap hardwareMap, String deviceName) {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        mSensor = hardwareMap.get(BNO055IMU.class, deviceName);
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        mSensor.initialize(imuParameters);
    }

    @Override
    protected double getSensorValue() {
        return mSensor.getAngularOrientation().firstAngle;
    }
}
