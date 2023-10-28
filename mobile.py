from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.button import Button
import socket
import threading
import time

cmd_port = 12345
ctrl_port = 54321

MCU_host = "192.168.149.101"

class CommandSenderApp(App):
    def build(self):
        self.layout = BoxLayout(orientation='vertical')
        self.label = Label(text="Press 'M' for ARM, 'L' for LAND, 'T' for TAKEOFF")
        self.layout.add_widget(self.label)

        # Log Area
        self.log_area = TextInput(readonly=True, size_hint=(1, 0.6), text="Log Area:\n")
        self.layout.add_widget(self.log_area)

        # Custom Command Field
        self.custom_command_label = Label(text="Custom Command:")
        self.layout.add_widget(self.custom_command_label)
        self.custom_command_entry = TextInput()
        self.layout.add_widget(self.custom_command_entry)
        self.send_custom_command_button = Button(text="Send Custom Command")
        self.send_custom_command_button.bind(on_press=self.send_custom_command)
        self.layout.add_widget(self.send_custom_command_button)

        # Buttons for commands
        self.arm_button = Button(text="ARM")
        self.arm_button.bind(on_press=lambda instance: self.send_command('ARMALL()'))
        self.land_button = Button(text="LAND")
        self.land_button.bind(on_press=lambda instance: self.send_command('LANDALL()'))
        self.takeoff_button = Button(text="TAKEOFF")
        self.takeoff_button.bind(on_press=lambda instance: self.send_command('TAKEOFFALL()'))
        self.layout.add_widget(self.arm_button)
        self.layout.add_widget(self.land_button)
        self.layout.add_widget(self.takeoff_button)

        return self.layout

    def send_command(self, command):
        threading.Thread(target=self.send, args=(MCU_host, command)).start()
        self.update_log(f'Sending command: {command}')

    def send_custom_command(self, instance):
        custom_command = self.custom_command_entry.text
        threading.Thread(target=self.send, args=(MCU_host, custom_command)).start()
        self.update_log(f'Sending custom command: {custom_command}')

    def update_log(self, message):
        current_log = self.log_area.text
        self.log_area.text = f"{current_log}{message}\n"

    def send(self, remote_host, command):
        client_socket = socket.socket()
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            client_socket.connect((remote_host, cmd_port))
        except socket.error as error_msg:
            self.update_log('{} - Caught exception : {}'.format(time.ctime(), error_msg))
            return
        client_socket.send(command.encode())

if __name__ == '__main__':
    CommandSenderApp().run()
