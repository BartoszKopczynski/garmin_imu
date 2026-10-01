import Toybox.WatchUi;

class test_imuDelegate extends WatchUi.BehaviorDelegate {

    var view;

    function initialize(view) {
        BehaviorDelegate.initialize();

        self.view = view;
    }

    function onKey(keyEvent) {

        var key = keyEvent.getKey();

        if (key == WatchUi.KEY_UP) {
            view.previousDisplayMode();
            return true;
        }

        if (key == WatchUi.KEY_DOWN) {
            view.nextDisplayMode();
            return true;
        }

        return false;
    }
}
