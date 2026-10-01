import Toybox.Graphics;
import Toybox.Sensor;
import Toybox.WatchUi;

class test_imuView extends WatchUi.View {

    // Number of samples used for moving average.
    // At 25 Hz, 5 samples = approximately 200 ms.
    const FILTER_SAMPLES = 5;

    var accelX = 0.0;
    var accelY = 0.0;
    var accelZ = 0.0;

    var gyroX = 0.0;
    var gyroY = 0.0;
    var gyroZ = 0.0;

    var magX = 0.0;
    var magY = 0.0;
    var magZ = 0.0;

    function initialize() {
        View.initialize();
    }

    function onShow() as Void {
        var options = {
            :period => 1,
            :accelerometer => {
                :enabled => true,
                :sampleRate => 25,
                :includePower => false,
                :includePitch => false,
                :includeRoll => false,
                :includeTimestamps => false
            },
            :gyroscope => {
                :enabled => true,
                :sampleRate => 25,
                :includeTimestamps => false
            },
            :magnetometer => {
                :enabled => true,
                :sampleRate => 25,
                :includeTimestamps => false
            },
            :heartBeatIntervals => {
                :enabled => false
            },
            // Magnetometer data is not supported in synchronous mode.
            :synchronous => false
        };

        Sensor.registerSensorDataListener(method(:onSensorData), options);
    }

    function onHide() as Void {
        Sensor.unregisterSensorDataListener();
    }

    // Calculate the average of the most recent samples in an array.
    function average(values) {
        if (values == null || values.size() == 0) {
            return 0.0;
        }

        var count = values.size();

        if (count > FILTER_SAMPLES) {
            count = FILTER_SAMPLES;
        }

        var start = values.size() - count;
        var sum = 0.0;

        for (var i = start; i < values.size(); i++) {
            sum += values[i];
        }

        return sum / count;
    }

    function onSensorData(data as Sensor.SensorData) as Void {
        // ---------------------------------------------------------
        // Accelerometer
        // Units: milli-G (mg)
        // ---------------------------------------------------------

        if (data.accelerometerData != null) {
            var accel = data.accelerometerData;

            if (accel.x.size() > 0) {
                accelX = average(accel.x);
                accelY = average(accel.y);
                accelZ = average(accel.z);
            }
        }

        // ---------------------------------------------------------
        // Gyroscope
        // Units: degrees/second
        // ---------------------------------------------------------

        if (data.gyroscopeData != null) {
            var gyro = data.gyroscopeData;

            if (gyro.x.size() > 0) {
                gyroX = average(gyro.x);
                gyroY = average(gyro.y);
                gyroZ = average(gyro.z);
            }
        }

        // ---------------------------------------------------------
        // Magnetometer
        // Units: milliGauss (mG)
        // ---------------------------------------------------------

        if (data.magnetometerData != null) {
            var mag = data.magnetometerData;

            if (mag.x.size() > 0) {
                magX = average(mag.x);
                magY = average(mag.y);
                magZ = average(mag.z);
            }
        }

        WatchUi.requestUpdate();
    }

    function onUpdate(dc as Dc) as Void {
        dc.setColor(Graphics.COLOR_BLACK, Graphics.COLOR_BLACK);
        dc.clear();

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        var width = dc.getWidth();
        var centerX = width / 2;

        // Title
        dc.drawText(centerX, 5, Graphics.FONT_LARGE, "IMU", Graphics.TEXT_JUSTIFY_CENTER);

        // Magnetometer - left column
        dc.drawText(65, 42, Graphics.FONT_SMALL, "MAG", Graphics.TEXT_JUSTIFY_CENTER);
        dc.drawText(65, 63, Graphics.FONT_XTINY, "X: " + magX.format("%d"), Graphics.TEXT_JUSTIFY_CENTER);
        dc.drawText(65, 80, Graphics.FONT_XTINY, "Y: " + magY.format("%d"), Graphics.TEXT_JUSTIFY_CENTER);
        dc.drawText(65, 97, Graphics.FONT_XTINY, "Z: " + magZ.format("%d"), Graphics.TEXT_JUSTIFY_CENTER);

        // Gyroscope - right column
        dc.drawText(width - 65, 42, Graphics.FONT_SMALL, "GYRO", Graphics.TEXT_JUSTIFY_CENTER);
        dc.drawText(width - 65, 63, Graphics.FONT_XTINY, "X: " + gyroX.format("%.1f"), Graphics.TEXT_JUSTIFY_CENTER);
        dc.drawText(width - 65, 80, Graphics.FONT_XTINY, "Y: " + gyroY.format("%.1f"), Graphics.TEXT_JUSTIFY_CENTER);
        dc.drawText(width - 65, 97, Graphics.FONT_XTINY, "Z: " + gyroZ.format("%.1f"), Graphics.TEXT_JUSTIFY_CENTER);

        // Accelerometer - centered
        dc.drawText(centerX, 125, Graphics.FONT_SMALL, "ACCEL", Graphics.TEXT_JUSTIFY_CENTER);
        dc.drawText(centerX, 146, Graphics.FONT_XTINY, "X: " + accelX.format("%d"), Graphics.TEXT_JUSTIFY_CENTER);
        dc.drawText(centerX, 163, Graphics.FONT_XTINY, "Y: " + accelY.format("%d"), Graphics.TEXT_JUSTIFY_CENTER);
        dc.drawText(centerX, 180, Graphics.FONT_XTINY, "Z: " + accelZ.format("%d"), Graphics.TEXT_JUSTIFY_CENTER);
    }

}
