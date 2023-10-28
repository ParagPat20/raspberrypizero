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
d = 'MCU'

class CommandSenderApp(App):
    def build(self):
        self.layout = BoxLayout(orientation='vertical')
        self.label = Label(text="Swarm Drone Control Using Mobile")
        self.layout.add_widget(self.label)

        # Output Terminal
        self.output_terminal = TextInput(multiline=True, readonly=True)
        self.layout.add_widget(self.output_terminal)

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
        self.arm_button.bind(on_press=lambda instance: self.send_command('m'))
        self.land_button = Button(text="LAND")
        self.land_button.bind(on_press=lambda instance: self.send_command('l'))
        self.takeoff_button = Button(text="TAKEOFF")
        self.takeoff_button.bind(on_press=lambda instance: self.send_command('t'))
        self.layout.add_widget(self.arm_button)
        self.layout.add_widget(self.land_button)
        self.layout.add_widget(self.takeoff_button)

        # WASD buttons
        self.wasd_label = Label(text="Use WASDUJ for control:")
        self.layout.add_widget(self.wasd_label)
        self.ctrl_buttons_layout = BoxLayout(orientation='horizontal')
        self.w_button = Button(text="W")
        self.a_button = Button(text="A")
        self.s_button = Button(text="S")
        self.d_button = Button(text="D")
        self.u_button = Button(text="U")
        self.j_button = Button(text="J")
        self.ctrl_buttons_layout.add_widget(self.w_button)
        self.ctrl_buttons_layout.add_widget(self.a_button)
        self.ctrl_buttons_layout.add_widget(self.s_button)
        self.ctrl_buttons_layout.add_widget(self.d_button)
        self.ctrl_buttons_layout.add_widget(self.u_button)
        self.ctrl_buttons_layout.add_widget(self.j_button)
        self.layout.add_widget(self.ctrl_buttons_layout)
        self.w_button.bind(on_press=lambda instance: self.send_wasd('w'))
        self.a_button.bind(on_press=lambda instance: self.send_wasd('a'))
        self.s_button.bind(on_press=lambda instance: self.send_wasd('s'))
        self.d_button.bind(on_press=lambda instance: self.send_wasd('d'))
        self.u_button.bind(on_press=lambda instance: self.send_wasd('u'))
        self.j_button.bind(on_press=lambda instance: self.send_wasd('j'))

        # Buttons to set 'd' variable
        self.mcu_button = Button(text="MCU")
        self.cd1_button = Button(text="CD1")
        self.cd2_button = Button(text="CD2")
        self.cd3_button = Button(text="CD3")
        self.cd4_button = Button(text="CD4")
        self.cd5_button = Button(text="CD5")
        self.cd6_button = Button(text="CD6")

        self.mcu_button.bind(on_press=lambda instance: self.set_d('MCU'))
        self.cd1_button.bind(on_press=lambda instance: self.set_d('CD1'))
        self.cd2_button.bind(on_press=lambda instance: self.set_d('CD2'))
        self.cd3_button.bind(on_press=lambda instance: self.set_d('CD3'))
        self.cd4_button.bind(on_press=lambda instance: self.set_d('CD4'))
        self.cd5_button.bind(on_press=lambda instance: self.set_d('CD5'))
        self.cd6_button.bind(on_press=lambda instance: self.set_d('CD6'))

        self.layout.add_widget(self.mcu_button)
        self.layout.add_widget(self.cd1_button)
        self.layout.add_widget(self.cd2_button)
        self.layout.add_widget(self.cd3_button)
        self.layout.add_widget(self.cd4_button)
        self.layout.add_widget(self.cd5_button)
        self.layout.add_widget(self.cd6_button)

        return self.layout

    def set_d(self, value):
        global d
        d = value
    def send_command(self, command):
        threading.Thread(target=self.send, args=(MCU_host, command)).start()

    def print_to_terminal(self, message):
        self.output_terminal.text += message + "\n"

    def send_wasd(self, key):
        ctrl_str = '0,0,0'
        if key == 'w':
            ctrl_str = '0.5,0,0'
        elif key == 'a':
            ctrl_str = '0,-0.5,0'
        elif key == 's':
            ctrl_str = '-0.5,0,0'
        elif key == 'd':
            ctrl_str = '0,0.5,0'
        elif key == 'u':
            ctrl_str = '0,0,-0.5'
        elif key == 'j':
            ctrl_str = '0,0,0.5'

        threading.Thread(target=self.send_ctrl, args=(ctrl_str,)).start()

    def send_custom_command(self, instance):
        custom_command = self.custom_command_entry.text
        threading.Thread(target=self.send, args=(MCU_host, custom_command)).start()

    def send(self, remote_host, command):
        client_socket = socket.socket()
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            client_socket.connect((remote_host, cmd_port))
            client_socket.send(command.encode())
        except socket.error as error_msg:
            print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
            print('{} - CLIENT_send_immediate_command({}) is not executed!'.format(time.ctime(), command))
            return

    def send_ctrl(self, ctrl_str):
        client_socket = socket.socket()
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            client_socket.connect((MCU_host, ctrl_port))
            client_socket.send(ctrl_str.encode())
        except socket.error as error_msg:
            print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
            print('{} - ClientSendCtrl(MCU) is not executed!')
            return

if __name__ == '__main__':
    CommandSenderApp().run()
