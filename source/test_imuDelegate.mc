import Toybox.Lang;
import Toybox.WatchUi;

class test_imuDelegate extends WatchUi.BehaviorDelegate {

    function initialize() {
        BehaviorDelegate.initialize();
    }

    function onMenu() as Boolean {
        WatchUi.pushView(new Rez.Menus.MainMenu(), new test_imuMenuDelegate(), WatchUi.SLIDE_UP);
        return true;
    }

}