"""Start the browser control without motors, IMU or walking policy."""

import argparse
import time

from mini_bdx_runtime.web_controller import WebController


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--token", default="duck-test")
    parser.add_argument("--camera", action="store_true")
    args = parser.parse_args()

    controller = WebController(
        command_freq=20,
        port=args.port,
        token=args.token,
        camera=args.camera,
    )
    print("Test mode only: no motor commands are sent.")
    try:
        while True:
            commands, buttons, _, _ = controller.get_last_command()
            print(
                "commands=",
                commands.tolist(),
                "start/pause=",
                controller.consume_desired_paused(),
                "sprint=",
                buttons.LB.is_pressed,
                end="\r",
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped")

