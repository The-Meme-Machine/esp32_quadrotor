import asyncio
import websockets
import json
import curses
import time

# --- Configuration ---
WEBSOCKET_URI = "ws://192.168.4.1/telemetry"
# ---------------------

def draw_dashboard(stdscr, data, connection_status, last_update_time):
    """
    Renders the telemetry data to the curses screen.
    """
    stdscr.clear()
    stdscr.nodelay(True)  # Make getch non-blocking
    
    # Get screen dimensions
    max_y, max_x = stdscr.getmaxyx()

    # --- Header ---
    title = "--- FLIGHT TELEMETRY DASHBOARD ---"
    status_line = f"STATUS: {connection_status}"
    time_line = f"Last Update: {last_update_time:.2f}s ago"
    
    stdscr.addstr(0, (max_x - len(title)) // 2, title, curses.A_BOLD)
    
    # Connection status color
    if connection_status == "CONNECTED":
        status_color = curses.color_pair(1)  # Green
    elif connection_status == "CONNECTING...":
        status_color = curses.color_pair(3)  # Yellow
    else:
        status_color = curses.color_pair(2)  # Red
        
    stdscr.addstr(1, 1, status_line, status_color | curses.A_BOLD)
    stdscr.addstr(1, max_x - len(time_line) - 2, time_line)
    stdscr.addstr(2, 0, "=" * max_x)

    # --- Quit Message ---
    quit_msg = "Press 'q' to quit"
    stdscr.addstr(max_y - 1, (max_x - len(quit_msg)) // 2, quit_msg, curses.A_REVERSE)

    if not data:
        stdscr.addstr(5, (max_x - 19) // 2, "Waiting for data...")
        stdscr.refresh()
        return

    # --- Data Display ---
    col1_x = 2
    col2_x = max_x // 3 + 2
    col3_x = (max_x // 3) * 2 + 2
    
    row = 4

    # --- Status & Control ---
    stdscr.addstr(row, col1_x, "SYSTEM STATUS", curses.A_UNDERLINE)
    row += 1
    
    armed_str = "ARMED" if data.get('armed', False) else "DISARMED"
    armed_color = curses.color_pair(1) if data.get('armed', False) else curses.color_pair(2)
    stdscr.addstr(row, col1_x, f"Armed:       ")
    stdscr.addstr(row, col1_x + 13, f"{armed_str}", armed_color | curses.A_BOLD)
    row += 1
    
    stdscr.addstr(row, col1_x, f"Flight Mode: {data.get('flight_mode', 'N/A')}")
    row += 1
    stdscr.addstr(row, col1_x, f"Loop Time:   {data.get('loop_time_us', 'N/A')} µs")
    row += 2

    stdscr.addstr(row, col1_x, "CONTROL LOOPS", curses.A_UNDERLINE)
    row += 1
    stdscr.addstr(row, col1_x, f"Roll:     {data.get('roll', 0.0):>8.2f}")
    row += 1
    stdscr.addstr(row, col1_x, f"Pitch:    {data.get('pitch', 0.0):>8.2f}")
    row += 1
    stdscr.addstr(row, col1_x, f"Yaw:      {data.get('yaw', 0.0):>8.2f}")
    row += 1
    stdscr.addstr(row, col1_x, f"Throttle: {data.get('throttle', 0.0):>8.2f}")
    
    # --- IMU Data ---
    row = 4
    stdscr.addstr(row, col2_x, "IMU (GYRO)", curses.A_UNDERLINE)
    row += 1
    stdscr.addstr(row, col2_x, f"G_X: {data.get('g_x', 0.0):>8.2f}")
    row += 1
    stdscr.addstr(row, col2_x, f"G_Y: {data.get('g_y', 0.0):>8.2f}")
    row += 1
    stdscr.addstr(row, col2_x, f"G_Z: {data.get('g_z', 0.0):>8.2f}")
    row += 2
    
    stdscr.addstr(row, col2_x, "IMU (ACCEL)", curses.A_UNDERLINE)
    row += 1
    stdscr.addstr(row, col2_x, f"XL_X: {data.get('xl_x', 0.0):>8.2f}")
    row += 1
    stdscr.addstr(row, col2_x, f"XL_Y: {data.get('xl_y', 0.0):>8.2f}")
    row += 1
    stdscr.addstr(row, col2_x, f"XL_Z: {data.get('xl_z', 0.0):>8.2f}")

    # --- Channels & Throttle ---
    row = 4
    stdscr.addstr(row, col3_x, "RADIO CHANNELS", curses.A_UNDERLINE)
    row += 1
    stdscr.addstr(row, col3_x, f"CH1 (Roll):  {data.get('ch1', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"CH2 (Pitch): {data.get('ch2', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"CH3 (Thr):   {data.get('ch3', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"CH4 (Yaw):   {data.get('ch4', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"CH5 (Arm):   {data.get('ch5', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"CH6 (Mode):  {data.get('ch6', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"CH7:         {data.get('ch7', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"CH8 (Alt):   {data.get('ch8', 0):>5}")
    row += 2

    stdscr.addstr(row, col3_x, "MOTOR THROTTLE", curses.A_UNDERLINE)
    row += 1
    stdscr.addstr(row, col3_x, f"THR_1: {data.get('thr_1', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"THR_2: {data.get('thr_2', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"THR_3: {data.get('thr_3', 0):>5}")
    row += 1
    stdscr.addstr(row, col3_x, f"THR_4: {data.get('thr_4', 0):>5}")

    stdscr.refresh()


async def telemetry_client(stdscr):
    """
    Main async function to connect to WebSocket and update dashboard.
    """
    telemetry_data = {}
    connection_status = "CONNECTING..."
    last_message_time = time.time()

    # --- Curses setup ---
    curses.curs_set(0)  # Hide the cursor
    stdscr.nodelay(True)  # Don't block on getch
    
    # Initialize colors
    if curses.has_colors():
        curses.start_color()
        # Pair 1: Green text on black background
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        # Pair 2: Red text on black background
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
        # Pair 3: Yellow text on black background
        curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)

    while True:
        try:
            # Draw dashboard immediately with current status
            last_update_duration = time.time() - last_message_time
            draw_dashboard(stdscr, telemetry_data, connection_status, last_update_duration)
            
            async with websockets.connect(WEBSOCKET_URI, open_timeout=2, close_timeout=1) as websocket:
                connection_status = "CONNECTED"
                last_message_time = time.time()
                
                while True:
                    # Check for quit key
                    if stdscr.getch() == ord('q'):
                        return

                    try:
                        # Wait for a message with a timeout
                        message = await asyncio.wait_for(websocket.recv(), timeout=0.1)
                        telemetry_data = json.loads(message)
                        last_message_time = time.time()
                        last_update_duration = 0.0

                    except asyncio.TimeoutError:
                        # No message received, just loop and check for quit key
                        last_update_duration = time.time() - last_message_time
                        pass
                    except (json.JSONDecodeError, TypeError):
                        connection_status = "DATA ERROR"
                        # Keep old data
                        
                    # Re-draw on every loop to keep time updated
                    draw_dashboard(stdscr, telemetry_data, connection_status, last_update_duration)

        except (websockets.exceptions.ConnectionClosedError, 
                websockets.exceptions.InvalidURI, 
                ConnectionRefusedError, 
                asyncio.TimeoutError,
                OSError) as e:
            connection_status = "DISCONNECTED"
            telemetry_data = {} # Clear data on disconnect
            last_update_duration = time.time() - last_message_time
            draw_dashboard(stdscr, telemetry_data, connection_status, last_update_duration)
            
            # Check for quit key while disconnected
            if stdscr.getch() == ord('q'):
                return
            
            await asyncio.sleep(1) # Wait 1 second before trying to reconnect

        except Exception as e:
            # Handle other unexpected errors
            stdscr.clear()
            stdscr.addstr(0, 0, f"An unexpected error occurred: {e}")
            stdscr.addstr(2, 0, "Press 'q' to exit.")
            stdscr.refresh()
            while stdscr.getch() != ord('q'):
                await asyncio.sleep(0.1)
            return

def main(stdscr):
    """
    Main wrapper to run the asyncio event loop.
    """
    try:
        asyncio.run(telemetry_client(stdscr))
    except (curses.error, KeyboardInterrupt):
        # Gracefully handle exit
        pass

if __name__ == "__main__":
    # curses.wrapper handles screen initialization and cleanup
    curses.wrapper(main)
    print("Telemetry dashboard closed.")
