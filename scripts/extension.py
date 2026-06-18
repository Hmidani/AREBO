import omni.ext
import omni.appwindow
import carb

class MyRobotExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("======Good ======")
       
       
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
       
       
        self._keyboard_device = self._input.get_keyboard_device_by_index(0)
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard_device, self._on_keyboard_event
        )

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            key_name = event.input.name
            print(f"Alpha: {key_name}")
           
     
            if key_name == "UP" or key_name == "W":
                print("move forward")
               
        return True

    def on_shutdown(self):
        print("====== Close ======")
        self._input.unsubscribe_to_keyboard_events(self._keyboard_device, self._sub_keyboard)