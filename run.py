import subprocess
import threading
from drone import *

context = zmq.Context(10)  # Allow up to 10 concurrent sockets
msg_socket = context.socket(zmq.PULL)
msg_socket.bind("tcp://*:5588")
msg_socket.setsockopt(zmq.RCVHWM, 1000)  # High water mark for incoming messages

poller = zmq.Poller()
poller.register(msg_socket, zmq.POLLIN)  # Monitor for incoming messages

def execute_command(immediate_command_str):
    try:
        log("Executing command: {}".format(repr(immediate_command_str)))

        # Use subprocess.run to capture output and error streams
        result = subprocess.run(
            immediate_command_str,
            shell=True,  # Allow shell features like pipes and redirection
            check=True,  # Raise an exception if the command fails
            stdout=subprocess.PIPE,  # Capture standard output
            stderr=subprocess.PIPE,  # Capture standard error
            text=True,  # Capture output as text
            encoding="utf-8"  # Specify encoding for text output
        )

        log("Command output:\n{}".format(result.stdout))
        log("Command error output:\n{}".format(result.stderr))
        log('{} - Command executed successfully'.format(time.ctime()))

    except Exception as e:
        log('{} - Error in execute_command: {}'.format(time.ctime(), e))

while True:
    socks = dict(poller.poll())

    if msg_socket in socks and socks[msg_socket] == zmq.POLLIN:
        try:
            immediate_command_str = msg_socket.recv(zmq.NOBLOCK)
            immediate_command_str = immediate_command_str.decode()
            command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
            command_thread.start()

        except zmq.error.Again:  # Handle non-blocking recv errors
            pass  # Wait for next poll event

        except zmq.ZMQError as zmq_error:
            log("ZMQ Error: {}".format(zmq_error))
            msg_socket.close()  # Recreate socket on ZMQ errors
            msg_socket = context.socket(zmq.PULL)
            msg_socket.bind("tcp://*:5588")
            poller.register(msg_socket, zmq.POLLIN)

        except Exception as e:
            log("Error: {}".fromat(e))
