package us.jcole.periodicsensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

abstract class PeriodicSensor {
    private Thread mThread;
    private double mValue;
    private ElapsedTime mAge;
    private ElapsedTime mCollectionTimer;
    private double mLastCollectionTime;
    private boolean mPaused = false;
    private boolean mRunning = true;

    protected abstract void initializeSensor(final HardwareMap hardwareMap, final String deviceName);
    protected abstract double getSensorValue();

    PeriodicSensor(final HardwareMap hardwareMap, final String deviceName, final int collectionPeriod) {
        mCollectionTimer = new ElapsedTime();
        mAge = new ElapsedTime();

        mThread = new Thread(new Runnable() {
            @Override
            public void run() {
                initializeSensor(hardwareMap, deviceName);

                Timer timer = new Timer();
                timer.scheduleAtFixedRate(new TimerTask() {
                    @Override
                    public void run() {
                        if (mPaused)
                            return;
                        mCollectionTimer.reset();
                        mValue = getSensorValue();
                        mAge.reset();
                        mLastCollectionTime = mCollectionTimer.milliseconds();
                    }
                }, 0, collectionPeriod);

                try {
                    while (mRunning) {
                        Thread.sleep(1);
                    }
                } catch (InterruptedException ie) {
                    // No need to retry or anything, since we're looping.
                }

                timer.purge();
                timer.cancel();
            }
        });

        mThread.start();
    }

    public void stop() {
        mRunning = false;
    }

    public double getValue() {
        return mValue;
    }

    public double getAge() {
        return mAge.milliseconds();
    }

    public double getLastCollectionTime() {
        return mLastCollectionTime;
    }

    public boolean isPaused() {
        return mPaused;
    }

    public void setPaused(boolean paused) {
        mPaused = paused;
    }

    public void pause() {
        setPaused(true);
    }

    public void resume() {
        setPaused(false);
    }
}
