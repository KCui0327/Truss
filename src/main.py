from mobile.webserver.server import app, socketio
import subprocess
import os

def main():
    control_proc = subprocess.Popen(['python3', 'control/control.py'])
    if control_proc.poll() is not None:
        print("Control process terminated unexpectedly.")
        return

    os.environ['CONTROL_PROC_PID'] = str(control_proc.pid)

    socketio.run(app, host='0.0.0.0', port=12345, debug=True)

if __name__ == '__main__':
    main()