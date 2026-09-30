import Toybox.Graphics;
import Toybox.WatchUi;

class test_imuView extends WatchUi.View {

    function initialize() {
        View.initialize();
    }

    function onUpdate(dc as Dc) as Void {
        dc.setColor(Graphics.COLOR_BLACK, Graphics.COLOR_BLACK);
        dc.clear();

        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);

        dc.drawText(dc.getWidth() / 2, 30, Graphics.FONT_LARGE, "IMU TEST", Graphics.TEXT_JUSTIFY_CENTER);

        dc.drawText(20, 100, Graphics.FONT_MEDIUM, "Accelerometer", Graphics.TEXT_JUSTIFY_LEFT);

        dc.drawText(20, 140, Graphics.FONT_SMALL, "X:  0", Graphics.TEXT_JUSTIFY_LEFT);

        dc.drawText(20, 165, Graphics.FONT_SMALL, "Y:  0", Graphics.TEXT_JUSTIFY_LEFT);

        dc.drawText(20, 190, Graphics.FONT_SMALL, "Z:  0", Graphics.TEXT_JUSTIFY_LEFT);
    }
}
