import Toybox.Graphics;
import Toybox.Math;
import Toybox.Sensor;
import Toybox.WatchUi;

class test_imuView extends WatchUi.View {

    const MODE_DATA = 0;
    const MODE_MIXED = 1;
    const MODE_3D = 2;

    var displayMode = MODE_MIXED;

    // ---------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------

    const SENSOR_RATE = 25.0;

    // The sensor listener supplies approximately one second
    // of samples at a time.
    const DEFAULT_DT = 1.0 / SENSOR_RATE;

    // How strongly the absolute accel/mag attitude corrects
    // the gyro-integrated quaternion at each sensor callback.
    //
    // Larger = less gyro drift, but more sensitivity to
    // accelerometer/magnetometer disturbances.
    const ATTITUDE_CORRECTION = 0.18;

    // Ignore accel values that are far from 1G.
    // This prevents strong linear acceleration from being
    // interpreted as gravity.
    const ACCEL_MIN = 650.0;
    const ACCEL_MAX = 1350.0;

    // Number of newest accel/mag samples used for the
    // absolute reference attitude.
    const REFERENCE_SAMPLES = 4;

    // ---------------------------------------------------------
    // Raw / filtered sensor values
    // ---------------------------------------------------------

    var accelX = 0.0;
    var accelY = 0.0;
    var accelZ = 0.0;

    var gyroX = 0.0;
    var gyroY = 0.0;
    var gyroZ = 0.0;

    var magX = 0.0;
    var magY = 0.0;
    var magZ = 0.0;

    // ---------------------------------------------------------
    // Quaternion
    //
    // q = [w, x, y, z]
    //
    // The quaternion rotates vectors from WATCH coordinates
    // into WORLD coordinates.
    // ---------------------------------------------------------

    var qw = 1.0;
    var qx = 0.0;
    var qy = 0.0;
    var qz = 0.0;

    var attitudeInitialized = false;

    var lastGyroTimestamp = null;

    // ---------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------

    function initialize() {
        View.initialize();
    }

    function setDisplayMode(mode) as Void {

        if (mode < MODE_DATA) {
            mode = MODE_3D;
        }

        if (mode > MODE_3D) {
            mode = MODE_DATA;
        }

        displayMode = mode;

        WatchUi.requestUpdate();
    }

    function getDisplayMode() {
        return displayMode;
    }

    function nextDisplayMode() as Void {

        displayMode += 1;

        if (displayMode > MODE_3D) {
            displayMode = MODE_DATA;
        }

        WatchUi.requestUpdate();
    }

    function previousDisplayMode() as Void {

        displayMode -= 1;

        if (displayMode < MODE_DATA) {
            displayMode = MODE_3D;
        }

        WatchUi.requestUpdate();
    }

    // ---------------------------------------------------------
    // SENSOR SETUP
    // ---------------------------------------------------------

    function onShow() as Void {

        // Reset attitude whenever the view becomes active.
        qw = 1.0;
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;

        attitudeInitialized = false;
        lastGyroTimestamp = null;

        var options = {
            :period => 1,

            :accelerometer => {
                :enabled => true,
                :sampleRate => 25,
                :includePower => false,
                :includePitch => false,
                :includeRoll => false,
                :includeTimestamps => true
            },

            :gyroscope => {
                :enabled => true,
                :sampleRate => 25,
                :includeTimestamps => true
            },

            :magnetometer => {
                :enabled => true,
                :sampleRate => 25,
                :includeTimestamps => true
            },

            :heartBeatIntervals => {
                :enabled => false
            },

            // Magnetometer is not supported in synchronous
            // sensor requests.
            :synchronous => false
        };

        Sensor.registerSensorDataListener(method(:onSensorData), options);
    }

    function onHide() as Void {
        Sensor.unregisterSensorDataListener();
    }

    // =========================================================
    // VECTOR MATH
    // =========================================================

    function vectorMagnitude(v) {

        return Math.sqrt(
            v[0] * v[0] +
            v[1] * v[1] +
            v[2] * v[2]
        );
    }

    function vectorNormalize(v) {

        var m = vectorMagnitude(v);

        if (m < 0.000001) {
            return [0.0, 0.0, 0.0];
        }

        return [
            v[0] / m,
            v[1] / m,
            v[2] / m
        ];
    }

    function vectorDot(a, b) {

        return a[0] * b[0] +
               a[1] * b[1] +
               a[2] * b[2];
    }

    function vectorCross(a, b) {

        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        ];
    }

    function vectorScale(v, s) {

        return [
            v[0] * s,
            v[1] * s,
            v[2] * s
        ];
    }

    function vectorSubtract(a, b) {

        return [
            a[0] - b[0],
            a[1] - b[1],
            a[2] - b[2]
        ];
    }

    // =========================================================
    // SENSOR FILTERING
    // =========================================================

    function averageTail(values) {

        if (values == null || values.size() == 0) {
            return 0.0;
        }

        var count = values.size();

        if (count > REFERENCE_SAMPLES) {
            count = REFERENCE_SAMPLES;
        }

        var start = values.size() - count;
        var sum = 0.0;

        for (var i = start; i < values.size(); i++) {
            sum += values[i];
        }

        return sum / count;
    }

    // =========================================================
    // QUATERNION NORMALIZATION
    // =========================================================

    function quaternionNormalize() as Void {

        var m = Math.sqrt(
            qw * qw +
            qx * qx +
            qy * qy +
            qz * qz
        );

        if (m < 0.000001) {
            qw = 1.0;
            qx = 0.0;
            qy = 0.0;
            qz = 0.0;
            return;
        }

        qw /= m;
        qx /= m;
        qy /= m;
        qz /= m;
    }

    // =========================================================
    // QUATERNION ROTATION
    //
    // Rotate a vector from WATCH coordinates to WORLD
    // coordinates.
    //
    // This assumes q is normalized.
    // =========================================================

    function quaternionRotate(v) {

        // t = 2 * cross(q.xyz, v)

        var tx = 2.0 * (
            qy * v[2] -
            qz * v[1]
        );

        var ty = 2.0 * (
            qz * v[0] -
            qx * v[2]
        );

        var tz = 2.0 * (
            qx * v[1] -
            qy * v[0]
        );

        return [
            v[0] +
            qw * tx +
            (qy * tz - qz * ty),

            v[1] +
            qw * ty +
            (qz * tx - qx * tz),

            v[2] +
            qw * tz +
            (qx * ty - qy * tx)
        ];
    }

    // =========================================================
    // GYRO INTEGRATION
    //
    // Gyro is in degrees/sec.
    //
    // Angular velocity is expressed in the WATCH coordinate
    // system, so qdot = 0.5 * q * omega.
    // =========================================================

    function integrateGyro(gxDeg, gyDeg, gzDeg, dt) as Void {

        var gx = Math.toRadians(gxDeg);
        var gy = Math.toRadians(gyDeg);
        var gz = Math.toRadians(gzDeg);

        var halfDt = 0.5 * dt;

        var dw = (-qx * gx - qy * gy - qz * gz) * halfDt;

        var dx = (qw * gx + qy * gz - qz * gy) * halfDt;

        var dy = (qw * gy - qx * gz + qz * gx) * halfDt;

        var dz = (qw * gz + qx * gy - qy * gx) * halfDt;

        qw += dw;
        qx += dx;
        qy += dy;
        qz += dz;

        quaternionNormalize();
    }

    // =========================================================
    // QUATERNION FROM WORLD-AXES-IN-BODY
    //
    // We know:
    //
    //   eastBody  = WORLD East expressed in WATCH coordinates
    //   northBody = WORLD North expressed in WATCH coordinates
    //   upBody    = WORLD Up expressed in WATCH coordinates
    //
    // These become the rows of the body -> world rotation
    // matrix.
    // =========================================================

    function quaternionFromReference(eastBody, northBody, upBody) {

        var m00 = eastBody[0];
        var m01 = eastBody[1];
        var m02 = eastBody[2];

        var m10 = northBody[0];
        var m11 = northBody[1];
        var m12 = northBody[2];

        var m20 = upBody[0];
        var m21 = upBody[1];
        var m22 = upBody[2];

        var trace = m00 + m11 + m22;

        var w;
        var x;
        var y;
        var z;

        if (trace > 0.0) {

            var s = Math.sqrt(trace + 1.0) * 2.0;

            w = 0.25 * s;
            x = (m21 - m12) / s;
            y = (m02 - m20) / s;
            z = (m10 - m01) / s;

        } else if (m00 > m11 && m00 > m22) {

            var s = Math.sqrt(1.0 + m00 - m11 - m22) * 2.0;

            w = (m21 - m12) / s;
            x = 0.25 * s;
            y = (m01 + m10) / s;
            z = (m02 + m20) / s;

        } else if (m11 > m22) {

            var s = Math.sqrt(1.0 + m11 - m00 - m22) * 2.0;

            w = (m02 - m20) / s;
            x = (m01 + m10) / s;
            y = 0.25 * s;
            z = (m12 + m21) / s;

        } else {

            var s = Math.sqrt(1.0 + m22 - m00 - m11) * 2.0;

            w = (m10 - m01) / s;
            x = (m02 + m20) / s;
            y = (m12 + m21) / s;
            z = 0.25 * s;
        }

        return [
            w,
            x,
            y,
            z
        ];
    }

    // =========================================================
    // QUATERNION BLENDING
    //
    // Normalized linear interpolation is sufficient here.
    // =========================================================

    function correctQuaternion(target, amount) as Void {

        var tw = target[0];
        var tx = target[1];
        var ty = target[2];
        var tz = target[3];

        // q and -q represent the same orientation.
        // Pick the shorter interpolation path.

        var d = qw * tw + qx * tx + qy * ty + qz * tz;

        if (d < 0.0) {
            tw = -tw;
            tx = -tx;
            ty = -ty;
            tz = -tz;
        }

        var inverse = 1.0 - amount;

        qw = qw * inverse + tw * amount;
        qx = qx * inverse + tx * amount;
        qy = qy * inverse + ty * amount;
        qz = qz * inverse + tz * amount;

        quaternionNormalize();
    }

    // =========================================================
    // BUILD ABSOLUTE ATTITUDE FROM ACCEL + MAG
    // =========================================================

    function calculateReferenceQuaternion() {

        var acceleration = [
            accelX,
            accelY,
            accelZ
        ];

        var magnetic = [
            magX,
            magY,
            magZ
        ];

        var accelerationMagnitude = vectorMagnitude(acceleration);

        // When stationary, accel should be close to 1000 mg.
        //
        // During strong movement, do not trust it as gravity.

        if (accelerationMagnitude < ACCEL_MIN || accelerationMagnitude > ACCEL_MAX) {
            return null;
        }

        // -----------------------------------------------------
        // WORLD UP expressed in WATCH coordinates.
        // -----------------------------------------------------

        var upBody = vectorNormalize(acceleration);

        // -----------------------------------------------------
        // Remove the vertical component from the magnetometer.
        //
        // Result = horizontal magnetic north.
        // -----------------------------------------------------

        var magneticVertical = vectorDot(magnetic, upBody);

        var northBody = vectorSubtract(magnetic, vectorScale(upBody, magneticVertical));

        var northMagnitude = vectorMagnitude(northBody);

        if (northMagnitude < 1.0) {
            return null;
        }

        northBody = vectorNormalize(northBody);

        // -----------------------------------------------------
        // WORLD EAST.
        //
        // East = North × Up
        // -----------------------------------------------------

        var eastBody = vectorNormalize(vectorCross(northBody, upBody));

        if (vectorMagnitude(eastBody) < 0.5) {
            return null;
        }

        // Re-orthogonalize north.
        northBody = vectorNormalize(vectorCross(upBody, eastBody));

        return quaternionFromReference(eastBody, northBody, upBody);
    }

    // =========================================================
    // SENSOR CALLBACK
    // =========================================================

    function onSensorData(data as Sensor.SensorData) as Void {

        var gyro = null;

        // -----------------------------------------------------
        // Accelerometer
        // -----------------------------------------------------

        if (data.accelerometerData != null) {

            var accel = data.accelerometerData;

            if (accel.x.size() > 0) {

                accelX = averageTail(accel.x);
                accelY = averageTail(accel.y);
                accelZ = averageTail(accel.z);
            }
        }

        // -----------------------------------------------------
        // Magnetometer
        // -----------------------------------------------------

        if (data.magnetometerData != null) {

            var mag = data.magnetometerData;

            if (mag.x.size() > 0) {

                magX = averageTail(mag.x);
                magY = averageTail(mag.y);
                magZ = averageTail(mag.z);
            }
        }

        // -----------------------------------------------------
        // Gyroscope
        //
        // IMPORTANT:
        // Do NOT average these samples.
        //
        // We integrate every gyro sample sequentially.
        // -----------------------------------------------------

        if (data.gyroscopeData != null) {
            gyro = data.gyroscopeData;
        }

        // -----------------------------------------------------
        // Absolute reference from accel + mag
        // -----------------------------------------------------

        var referenceQuaternion = calculateReferenceQuaternion();

        // -----------------------------------------------------
        // First callback:
        //
        // Initialize orientation directly from accel+mag.
        // We do NOT integrate the first gyro batch because its
        // timestamps represent samples preceding this initial
        // absolute alignment.
        // -----------------------------------------------------

        if (!attitudeInitialized) {

            if (referenceQuaternion != null) {

                qw = referenceQuaternion[0];
                qx = referenceQuaternion[1];
                qy = referenceQuaternion[2];
                qz = referenceQuaternion[3];

                quaternionNormalize();

                attitudeInitialized = true;
            }

            if (gyro != null && gyro.timestamp != null && gyro.timestamp.size() > 0) {
                lastGyroTimestamp = gyro.timestamp[gyro.timestamp.size() - 1];
            }

            WatchUi.requestUpdate();
            return;
        }

        // =====================================================
        // INTEGRATE ALL GYRO SAMPLES
        // =====================================================

        if (gyro != null && gyro.x.size() > 0) {

            for (var i = 0; i < gyro.x.size(); i++) {

                var dt = DEFAULT_DT;

                if (gyro.timestamp != null && gyro.timestamp.size() > i) {

                    var timestamp = gyro.timestamp[i];

                    if (lastGyroTimestamp != null) {

                        dt = (timestamp - lastGyroTimestamp) / 1000.0;

                        // Protect against corrupted timing.
                        if (dt < 0.005) {
                            dt = 0.005;
                        }

                        if (dt > 0.20) {
                            dt = 0.20;
                        }
                    }

                    lastGyroTimestamp = timestamp;
                }

                gyroX = gyro.x[i];
                gyroY = gyro.y[i];
                gyroZ = gyro.z[i];

                integrateGyro(gyroX, gyroY, gyroZ, dt);
            }
        }

        // =====================================================
        // CORRECT DRIFT USING ACCEL + MAG
        // =====================================================

        if (referenceQuaternion != null) {

            correctQuaternion(referenceQuaternion, ATTITUDE_CORRECTION);
        }

        WatchUi.requestUpdate();
    }

    // =========================================================
    // WORLD -> SCREEN PROJECTION
    //
    // World:
    //
    //   X = East
    //   Y = North
    //   Z = Up
    //
    // We deliberately use a slightly oblique projection so
    // that the 3D nature of the vectors remains obvious on the
    // 2D circular display.
    // =========================================================

    function projectWorld(v, centerX, centerY, scaleValue) {

        var east = v[0];
        var north = v[1];
        var up = v[2];

        var screenX = centerX + (east + north * 0.38) * scaleValue;

        var screenY = centerY - (up - north * 0.25) * scaleValue;

        return [
            screenX,
            screenY
        ];
    }

    // =========================================================
    // DRAW ARROW
    // =========================================================

    function drawArrow(dc, p0, p1, label) as Void {

        dc.drawLine(p0[0], p0[1], p1[0], p1[1]);

        var dx = p1[0] - p0[0];
        var dy = p1[1] - p0[1];

        var length = Math.sqrt(dx * dx + dy * dy);

        if (length > 3.0) {

            var ux = dx / length;
            var uy = dy / length;

            var arrowLength = 7.0;
            var arrowWidth = 3.0;

            var leftX = p1[0] - ux * arrowLength - uy * arrowWidth;
            var leftY = p1[1] - uy * arrowLength + ux * arrowWidth;

            var rightX = p1[0] - ux * arrowLength + uy * arrowWidth;
            var rightY = p1[1] - uy * arrowLength - ux * arrowWidth;

            dc.drawLine(p1[0], p1[1], leftX, leftY);

            dc.drawLine(p1[0], p1[1], rightX, rightY);
        }

        dc.drawText(p1[0], p1[1] - 11, Graphics.FONT_XTINY, label, Graphics.TEXT_JUSTIFY_CENTER);
    }

    // =========================================================
    // DRAW WATCH PLANE
    //
    // Four corners of the watch face, transformed by q.
    // This makes the attitude much easier to understand than
    // three independent lines.
    // =========================================================

    function drawWatchPlane(dc, centerX, centerY) as Void {

        var half = 0.72;
        var scaleValue = 44.0;

        var localCorners = [
            [-half, -half, 0.0],
            [ half, -half, 0.0],
            [ half,  half, 0.0],
            [-half,  half, 0.0]
        ];

        // IMPORTANT:
        // Do not use:
        //
        //     var points = [];
        //
        // because an empty array literal is inferred as a
        // zero-length tuple by the Monkey C compiler.
        //
        // Allocate an actual four-element array instead.

        var points = new[4];

        points[0] = projectWorld(localCorners[0], centerX, centerY, scaleValue);
        points[1] = projectWorld(localCorners[1], centerX, centerY, scaleValue);
        points[2] = projectWorld(localCorners[2], centerX, centerY, scaleValue);
        points[3] = projectWorld(localCorners[3], centerX, centerY, scaleValue);

        // ---------------------------------------------------------
        // Outer virtual watch face
        // ---------------------------------------------------------

        dc.drawLine(points[0][0], points[0][1], points[1][0], points[1][1]);

        dc.drawLine(points[1][0], points[1][1], points[2][0], points[2][1]);

        dc.drawLine(points[2][0], points[2][1], points[3][0], points[3][1]);

        dc.drawLine(points[3][0], points[3][1], points[0][0], points[0][1]);

        // ---------------------------------------------------------
        // Center axes of the virtual watch
        // ---------------------------------------------------------

        var xAxis = quaternionRotate([1.0, 0.0, 0.0]);

        var yAxis = quaternionRotate([0.0, 1.0, 0.0]);

        var px0 = projectWorld(vectorScale(xAxis, -0.55), centerX, centerY, scaleValue);

        var px1 = projectWorld(vectorScale(xAxis, 0.55), centerX, centerY, scaleValue);

        var py0 = projectWorld(vectorScale(yAxis, -0.55), centerX, centerY, scaleValue);

        var py1 = projectWorld(vectorScale(yAxis, 0.55), centerX, centerY, scaleValue);

        dc.drawLine(px0[0], px0[1], px1[0], px1[1]);

        dc.drawLine(py0[0], py0[1], py1[0], py1[1]);
    }

    // =========================================================
    // DRAW 3D ATTITUDE
    // =========================================================

    function drawAttitude(dc, centerX, centerY) as Void {

        if (!attitudeInitialized) {

            dc.drawText(centerX, centerY - 5, Graphics.FONT_SMALL, "CALIBRATING", Graphics.TEXT_JUSTIFY_CENTER);
            //attitudeInitialized = true;//

            return;
        }

        // -----------------------------------------------------
        // Circular attitude frame
        // -----------------------------------------------------

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        dc.drawCircle(centerX, centerY, 72);

        // -----------------------------------------------------
        // Draw virtual watch face
        // -----------------------------------------------------

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        drawWatchPlane(dc, centerX, centerY);

        // -----------------------------------------------------
        // WATCH LOCAL AXES
        //
        // X = right side of watch
        // Y = 12 o'clock direction
        // Z = out of the watch face
        // -----------------------------------------------------

        var watchX = quaternionRotate([1.0, 0.0, 0.0]);

        var watchY = quaternionRotate([0.0, 1.0, 0.0]);

        var watchZ = quaternionRotate([0.0, 0.0, 1.0]);

        // -----------------------------------------------------
        // X AXIS
        // -----------------------------------------------------

        dc.setColor(Graphics.COLOR_RED, Graphics.COLOR_TRANSPARENT);

        var pX = projectWorld(watchX, centerX, centerY, 48.0);

        drawArrow(dc, [centerX, centerY], pX, "X");

        // -----------------------------------------------------
        // Y AXIS
        // -----------------------------------------------------

        dc.setColor(Graphics.COLOR_GREEN, Graphics.COLOR_TRANSPARENT);

        var pY = projectWorld(watchY, centerX, centerY, 48.0);

        drawArrow(dc, [centerX, centerY], pY, "Y");

        // -----------------------------------------------------
        // Z AXIS
        //
        // This is the important vector:
        //
        // direction normal to the watch face.
        // -----------------------------------------------------

        dc.setColor(Graphics.COLOR_BLUE, Graphics.COLOR_TRANSPARENT);

        var pZ = projectWorld(watchZ, centerX, centerY, 58.0);

        drawArrow(dc, [centerX, centerY], pZ, "Z");

        // -----------------------------------------------------
        // Origin
        // -----------------------------------------------------

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        dc.drawCircle(centerX, centerY, 4);
    }

    // =========================================================
    // HEADING
    //
    // Assumption:
    // watch +Y points toward 12 o'clock.
    //
    // When the watch is flat:
    //
    // +Y = North
    // +X = East
    // +Z = Up
    // =========================================================

    function getHeading() {

        var forward = quaternionRotate([0.0, 1.0, 0.0]);

        var heading = Math.toDegrees(Math.atan2(forward[0], forward[1]));

        if (heading < 0.0) {
            heading += 360.0;
        }

        return heading;
    }

    // =========================================================
    // SCREEN
    // =========================================================

    function drawDataMode(dc as Dc) as Void {

        var width = dc.getWidth();
        var centerX = width / 2;

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        dc.drawText(centerX, 12, Graphics.FONT_SMALL, "IMU DATA", Graphics.TEXT_JUSTIFY_CENTER);

        // ---------------------------------------------------------
        // ACCEL
        // ---------------------------------------------------------

        dc.drawText(centerX, 45, Graphics.FONT_SMALL, "ACCEL", Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 65, Graphics.FONT_XTINY, "X: " + accelX.format("%.0f"), Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 80, Graphics.FONT_XTINY, "Y: " + accelY.format("%.0f"), Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 95, Graphics.FONT_XTINY, "Z: " + accelZ.format("%.0f"), Graphics.TEXT_JUSTIFY_CENTER);

        // ---------------------------------------------------------
        // GYRO
        // ---------------------------------------------------------

        dc.drawText(centerX, 120, Graphics.FONT_SMALL, "GYRO", Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 140, Graphics.FONT_XTINY, "X: " + gyroX.format("%.1f"), Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 155, Graphics.FONT_XTINY, "Y: " + gyroY.format("%.1f"), Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 170, Graphics.FONT_XTINY, "Z: " + gyroZ.format("%.1f"), Graphics.TEXT_JUSTIFY_CENTER);

        // ---------------------------------------------------------
        // MAG
        // ---------------------------------------------------------

        dc.drawText(centerX, 195, Graphics.FONT_SMALL, "MAG", Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 215, Graphics.FONT_XTINY, "X: " + magX.format("%.0f"), Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 230, Graphics.FONT_XTINY, "Y: " + magY.format("%.0f"), Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 245, Graphics.FONT_XTINY, "Z: " + magZ.format("%.0f"), Graphics.TEXT_JUSTIFY_CENTER);
    }

    function drawMixedMode(dc as Dc) as Void {

        var width = dc.getWidth();
        var centerX = width / 2;

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        dc.drawText(centerX, 5, Graphics.FONT_SMALL, "ATTITUDE", Graphics.TEXT_JUSTIFY_CENTER);

        // ---------------------------------------------------------
        // 3D attitude
        // ---------------------------------------------------------

        drawAttitude(dc, centerX, 93);

        // ---------------------------------------------------------
        // Heading
        // ---------------------------------------------------------

        dc.drawText(centerX, 170, Graphics.FONT_SMALL, "HDG " + getHeading().format("%.0f") + "°", Graphics.TEXT_JUSTIFY_CENTER);

        // ---------------------------------------------------------
        // Raw sensor data
        // ---------------------------------------------------------

        dc.drawText(centerX, 195, Graphics.FONT_XTINY, "ACC " + accelX.format("%.0f") + " " + accelY.format("%.0f") + " " + accelZ.format("%.0f"), Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 211, Graphics.FONT_XTINY, "GYR " + gyroX.format("%.1f") + " " + gyroY.format("%.1f") + " " + gyroZ.format("%.1f"), Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 227, Graphics.FONT_XTINY, "MAG " + magX.format("%.0f") + " " + magY.format("%.0f") + " " + magZ.format("%.0f"), Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(centerX, 252, Graphics.FONT_XTINY, "MODE 2/3", Graphics.TEXT_JUSTIFY_CENTER);
    }

    function draw3DMode(dc as Dc) as Void {

        var width = dc.getWidth();
        var height = dc.getHeight();

        var centerX = width / 2;
        var centerY = height / 2;

        // ---------------------------------------------------------
        // Large attitude indicator
        // ---------------------------------------------------------

        drawAttitudeFullscreen(dc, centerX, centerY);

        // ---------------------------------------------------------
        // Heading
        // ---------------------------------------------------------

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        dc.drawText(centerX, 225, Graphics.FONT_SMALL, getHeading().format("%.0f") + "°", Graphics.TEXT_JUSTIFY_CENTER);
    }

    function drawAttitudeFullscreen(dc, centerX, centerY) as Void {

        if (!attitudeInitialized) {

            dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

            dc.drawText(centerX, centerY, Graphics.FONT_SMALL, "CALIBRATING", Graphics.TEXT_JUSTIFY_CENTER);

            return;
        }

        // ---------------------------------------------------------
        // Large reference circle
        // ---------------------------------------------------------

        dc.setColor(Graphics.COLOR_DK_RED, Graphics.COLOR_TRANSPARENT);
        dc.drawCircle(centerX, centerY, 140);

        dc.setColor(Graphics.COLOR_RED, Graphics.COLOR_TRANSPARENT);
        dc.drawCircle(centerX, centerY, 139);

        dc.setColor(Graphics.COLOR_ORANGE, Graphics.COLOR_TRANSPARENT);
        dc.drawCircle(centerX, centerY, 138);

        // dc.setColor(
        //     Graphics.COLOR_WHITE,
        //     Graphics.COLOR_TRANSPARENT
        // );

        // dc.drawText(
        //     centerX,
        //     centerY - 140,
        //     Graphics.FONT_XTINY,
        //     "145",
        //     Graphics.TEXT_JUSTIFY_CENTER
        // );

        // ---------------------------------------------------------
        // Virtual watch plane
        // ---------------------------------------------------------

        drawWatchPlaneLarge(dc, centerX, centerY);

        // ---------------------------------------------------------
        // Watch axes
        // ---------------------------------------------------------

        var watchX = quaternionRotate([1.0, 0.0, 0.0]);

        var watchY = quaternionRotate([0.0, 1.0, 0.0]);

        var watchZ = quaternionRotate([0.0, 0.0, 1.0]);

        // X
        dc.setColor(Graphics.COLOR_RED, Graphics.COLOR_TRANSPARENT);

        var pX = projectWorld(watchX, centerX, centerY, 82.0);

        drawArrow(dc, [centerX, centerY], pX, "X");

        // Y
        dc.setColor(Graphics.COLOR_GREEN, Graphics.COLOR_TRANSPARENT);

        var pY = projectWorld(watchY, centerX, centerY, 82.0);

        drawArrow(dc, [centerX, centerY], pY, "Y");

        // Z
        dc.setColor(Graphics.COLOR_BLUE, Graphics.COLOR_TRANSPARENT);

        var pZ = projectWorld(watchZ, centerX, centerY, 100.0);

        drawArrow(dc, [centerX, centerY], pZ, "Z");

        // Origin

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        dc.drawCircle(centerX, centerY, 4);
    }

    function drawWatchPlaneLarge(dc, centerX, centerY) as Void {

        var half = 0.85;
        var scaleValue = 72.0;

        var c0 = quaternionRotate([-half, -half, 0.0]);

        var c1 = quaternionRotate([half, -half, 0.0]);

        var c2 = quaternionRotate([half, half, 0.0]);

        var c3 = quaternionRotate([-half, half, 0.0]);

        var p0 = projectWorld(c0, centerX, centerY, scaleValue);

        var p1 = projectWorld(c1, centerX, centerY, scaleValue);

        var p2 = projectWorld(c2, centerX, centerY, scaleValue);

        var p3 = projectWorld(c3, centerX, centerY, scaleValue);

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        dc.drawLine(p0[0], p0[1], p1[0], p1[1]);

        dc.drawLine(p1[0], p1[1], p2[0], p2[1]);

        dc.drawLine(p2[0], p2[1], p3[0], p3[1]);

        dc.drawLine(p3[0], p3[1], p0[0], p0[1]);
    }

    function onUpdate(dc as Dc) as Void {

        dc.setColor(Graphics.COLOR_BLACK, Graphics.COLOR_BLACK);

        dc.clear();

        switch (displayMode) {

            case MODE_DATA:
                drawDataMode(dc);
                break;

            case MODE_MIXED:
                drawMixedMode(dc);
                break;

            case MODE_3D:
                draw3DMode(dc);
                break;
        }
    }
}
