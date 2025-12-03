import os
import subprocess
import sys
import time
import re
import select

import protocol_test

#Variables

SystemState = "systemState"
Led = "(LED.pGPIOx.ODR >> 5) & 0b1"
ElapsedTIme = "ElapsedTime"
TelemetryStatus = "telemetry_status"
HeartBeatStatus = "heartbeat_status"

#Breakpoints
#GDB scripts actions (To form TPs)
button1 = "button1.gdb"
button2 = "button2.gdb"
heartbeat = "heartbeat.gdb"
sampling = "sampling.gdb"
pwm = "sampling.gdb"
wait_command = "command.gdb"

#Track Breakpoints
SamplingBreakpoint = "TIM2_IRQHandler" 

#Commands

SystemEnable = '01'
SystemDisable = '02'
OpenLoop = '03'
CloseLoop = '04'


#GDB scripts function used
main_while = "main_while.gdb"
read_data = "data.gdb"

#Paths
GDB_SERVER = r"C:\ST\STM32CubeCLT_1.20.0\STLink-gdb-server\bin\ST-LINK_gdbserver.exe"
GDB_PATH = r"C:\ST\STM32CubeCLT_1.20.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe"
ELF_PATH = os.path.expandvars(r"%USERPROFILE%\Documents\InternProject\STM32F446RE\GRID_FORMING\debug\GRID_FORMING.elf")
TC_FOLDER = os.path.expandvars(r"%USERPROFILE%\Documents\InternProject\STM32F446RE\TestFrameWork")

def cmd_command(command):
    cmd = (
        'python -c "import protocol_test; '
        'import time; '
        'time.sleep(5); '
        'print(\'Opening Port\'); '
        'protocol_test.usartConfig(); '
        'protocol_test.readHeartBeat(); '
        'print(\'Receive HeartBeat\'); '
        f'protocol_test.command(\'{command}\'); '
        'print(\'Command Sent\')"'
    )
    subprocess.Popen(f'start "" cmd /c {cmd}', shell=True)

def start_gdb_server():
    cmd = [
        GDB_SERVER,
        "-cp", r"C:\ST\STM32CubeCLT_1.20.0\STM32CubeProgrammer\bin",
        "-i", "0669FF545383564867214231",
        "--halt", "--frequency", "4000", "-d"
    ]
    print("Starting GDB server...")
    return subprocess.Popen(cmd)

def parse_gdb_value(output, var_name):
    # Find all occurrences of "var_name = value"
    pattern = rf"{re.escape(var_name)}\s*=\s*(\d+)"
    matches = re.findall(pattern, output)
    if matches:
        return matches[-1]  # Return the last occurrence
    return None

def start_gdb_client():
    cmd = [GDB_PATH, ELF_PATH]
    process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    with open(os.path.join(TC_FOLDER, "init_load.gdb")) as f:
        commands = f.read()
    process.stdin.write(commands + "\n")
    process.stdin.flush()
    return process

def run_scripts(process, scripts, track_breakpoints):
    # Send scripts
    for script in scripts:
        with open(os.path.join(TC_FOLDER, script)) as f:
            commands = f.read()
        process.stdin.write(commands + "\n")
        process.stdin.flush()

    with open(os.path.join(TC_FOLDER, read_data)) as f:
            commands = f.read()
    process.stdin.write(commands + "\n")
    process.stdin.flush()
    
    # Send marker
    process.stdin.write('echo ---TP-END---\n')
    process.stdin.flush()

    # Read output until marker or timeout
    timestamps = {}
    output_lines = []
    start_time = time.time()
    timeout = 20  # Adjust based on script duration

    while True:
        if time.time() - start_time > timeout:
            print("Timeout waiting for marker!")
            break

        # Read all available data without blocking
        chunk = process.stdout.read(1)
        if chunk:
            output_lines.append(chunk)
            current_output = ''.join(output_lines)

            # Check for breakpoints
            if track_breakpoints:
                for bp in track_breakpoints:
                    if bp in current_output and bp not in timestamps:
                        timestamps[bp] = time.time()

            if '---TP-END---' in current_output:
                print("Found End Marker")
                break

        else:
            time.sleep(0.1)  # Avoid busy loop

    return ''.join(output_lines), timestamps

def close_gdb_client(process):
    process.stdin.write("disconnect\nquit\n")
    process.stdin.flush()    
    output, error = process.communicate()
    return output + error

def run_test_case(tc_name):
    tc_script = os.path.join(TC_FOLDER, f"{tc_name}.gdb")
    if not os.path.exists(tc_script):
        print(f"Test case script not found: {tc_script}")
        return

    cmd = [GDB_PATH, ELF_PATH, "-x", tc_script]
    print(f"Running: {' '.join(cmd)}")

    process = subprocess.run(cmd, capture_output=True, text=True)
    
    # Show output
    output = process.stdout + process.stderr
    print("=== GDB Output ===")
    print(output)
    print("==================\n")

    return output

def reset_board(process):
    process.stdin.write("monitor reset halt\n")
    process.stdin.flush()
    time.sleep(1)
    process.stdin.write("load\n")
    process.stdin.flush()
    time.sleep(1)
    process.stdin.write("delete breakpoints\n")
    process.stdin.flush()

    #Read until we see (gdb) prompt or timeout
    start_time = time.time()
    output_lines = []
    while time.time() - start_time < 5:
        line = process.stdout.readline()
        if not line:
            break
        output_lines.append(line)
        if "(gdb)" in line:
            break
    return "".join(output_lines)

def run_multiple_scripts(scripts):
    """Run multiple .gdb scripts in a single GDB session"""
    cmd = [GDB_PATH, "-batch", "-cd", TC_FOLDER]
    for script in scripts:
        cmd += ["-x", script]
    cmd += [ELF_PATH]
    
    print(f"Running {len(scripts)} scripts in single GDB session...")
    process = subprocess.run(cmd, capture_output=True, text=True)
    
    output = process.stdout + process.stderr
    print("=== GDB Output ===")
    print(output)
    print("==================\n")
    return output

def load_program():
    server = start_gdb_server()
    init_load()        #Load source code to board
    server.terminate()
    server.wait()
    time.sleep(2)

#Output #TP, Variables Names, Expected Values, Receive Values, Result, Error Message

def test_procedure(process, test_number, variables_name_list , expected_values_list, track_breakpoints, expected_time_breakpoints, scripts_list):
    output = [test_number, variables_name_list, expected_values_list, None, None, None]
    print(f"\n====================================TP-{test_number}====================================\n")
    out, timestamps = run_scripts(process, scripts_list, track_breakpoints)

    
    values = []
    if variables_name_list:
        for x in range(len(variables_name_list)):
            values.append(parse_gdb_value(out, variables_name_list[x]))
    if timestamps:
        for x in range(len(timestamps)):
            values.append(timestamps[x])
    output[3] = values
    output[4] = True
    
    if variables_name_list:
        for x in range(len(variables_name_list)):
            if not output[3][x] == str(expected_values_list[x]):
                output[4] = False
                break
    if timestamps:
        for x in range(len(timestamps)):
            if not output[3][range(len(variables_name_list))+x] == str(expected_time_breakpoints[x]):
                output[4] = False
                break
    output[1].append(track_breakpoints)
    output[2].append(expected_time_breakpoints)
    return output

def tp001(process):
    output = test_procedure(process, 1, [SystemState], [0], [], [], [main_while])
    return output

def tp002(process):
    output = test_procedure(process, 2, [Led], [0], [], [], [main_while])
    return output

def tp003(process):
    output = test_procedure(process, 3, [SystemState, Led], [0,0], [], [], [main_while, button1, button1])
    return output

def tp004(process):
    output = test_procedure(process, 4, [SystemState, Led], [0, 0], [], [], [main_while, button1, button2, button1])
    return output

def tp005(process):
    output = test_procedure(process, 5, [SystemState, Led], [1,1], [], [], [main_while, button1])
    return output

def tp006(process):
    output = test_procedure(process, 6, [SystemState, Led], [1,1], [], [], [main_while, button1])
    return output

def tp007(process):
    output = test_procedure(process, 7, [SystemState, Led], [1,1], [], [], [main_while, button1, button2, button2])
    return output

def tp008(process):
    output = test_procedure(process, 8, [SystemState, Led], [1,1], [], [], [main_while, button1, button2, button2])
    return output

def tp009(process):
    output = test_procedure(process, 9, [SystemState, Led], [3, 0], [], [], [main_while, button1, button2, heartbeat, heartbeat])
    return output

def tp010(process):
    output = test_procedure(process, 10, [SystemState, Led], [3, 0], [], [], [main_while, button1, button2, heartbeat, heartbeat])
    return output

#TO-DO
def tp011(process):
    output = test_procedure(process, 11, [], [], [SamplingBreakpoint], [0.0001041], [main_while, button1, button2, sampling, sampling])
    return output

#TO-DO
def tp012(process):
    output = test_procedure(process, 12, [SystemState, Led], [3, 0], [], [], [main_while, button1, button2, heartbeat, heartbeat])
    return output

#TO-DO
def tp013(process):
    output = test_procedure(process, 11, [SystemState, Led], [3, 0], [], [], [main_while, button1, button2, heartbeat, heartbeat])
    return output

def tp014(process):
    output = test_procedure(process, 14, [ElapsedTIme], [0], [], [], [main_while])
    return output

#TO-DO
def tp015(process):
    output = test_procedure(process, 15, [SystemState, Led], [3, 0], [], [], [main_while, button1, button2, heartbeat, heartbeat])
    return output

#TO-DO
def tp016(process):
    output = test_procedure(process, 15, [SystemState, Led], [3, 0], [], [], [main_while, button1, button2, heartbeat, heartbeat])
    return output

#TO-DO
def tp017(process):
    output = test_procedure(process, 15, [SystemState, Led], [3, 0], [], [], [main_while, button1, button2, heartbeat, heartbeat])
    return output

def tp018(process):
    output = test_procedure(process, 18, [TelemetryStatus], [0], [], [], [main_while])
    return output

#TO-DO
def tp019(process):
    output = test_procedure(process, 19, [TelemetryStatus], [0], [], [], [main_while])
    return output

#TO-DO
def tp020(process):
    output = test_procedure(process, 20, [TelemetryStatus], [0], [], [], [main_while])
    return output

#TO-DO
def tp021(process):
    output = test_procedure(process, 21, [TelemetryStatus], [0], [], [], [main_while])
    return output

#TO-DO
def tp022(process):
    output = test_procedure(process, 21, [TelemetryStatus], [0], [], [], [main_while])
    return output

def tp023(process):
    cmd_command(SystemDisable)
    output = test_procedure(process, 23, [SystemState], [0], [], [], [main_while, heartbeat, wait_command, main_while])
    return output

def tp024(process):
    cmd_command(SystemEnable)
    output = test_procedure(process, 24, [SystemState], [1], [], [], [main_while, button1, heartbeat, wait_command, main_while])
    return output

def tp025(process):
    cmd_command(SystemEnable)
    output = test_procedure(process, 25, [SystemState], [1], [], [], [main_while, heartbeat, wait_command, main_while])
    return output

def tp026(process):
    cmd_command(SystemDisable)
    output = test_procedure(process, 26, [SystemState], [0], [], [], [main_while, button1, heartbeat, wait_command, main_while])
    return output

def tp027(process):
    cmd_command(CloseLoop)
    output = test_procedure(process, 27, [SystemState], [3], [], [], [main_while, button1, button2, heartbeat, wait_command, main_while])
    return output

def tp028(process):
    cmd_command(OpenLoop)
    output = test_procedure(process, 28, [SystemState], [1], [], [], [main_while, button1, heartbeat, wait_command, main_while])
    return output

def tp029(process):
    cmd_command(OpenLoop)
    output = test_procedure(process, 29, [SystemState], [1], [], [], [main_while, button1, button2, heartbeat, wait_command, main_while])
    return output

def tp030(process):
    cmd_command(CloseLoop)
    output = test_procedure(process, 29, [SystemState], [3], [], [], [main_while, button1, heartbeat, wait_command, main_while])
    return output

lTPs = [tp001, tp002, tp003, tp004, tp005, tp006, tp007, tp008, tp009, tp010, tp011]

def testTPs(ltps):
    server = start_gdb_server()
    time.sleep(3)
    client = start_gdb_client()
    nTPs = len(ltps)
    fTPs = 0
    coverage = []
    y = 0
    for x in ltps:
        coverage.append(x(client))
        reset_board(client)
        if not coverage[y][4]:
            fTPs += 1
        y += 1
        time.sleep(1)

    close_gdb_client(client)
    server.terminate()
    server.wait()
    
    print("\n\n==============================================================================")
    print("===============================COVERAGE RESULT================================")
    print("==============================================================================")
    print("Number of TPs: ", nTPs, "   Fail TPs: ", fTPs)
    for z in range(len(coverage)):
        print("#TP ",coverage[z][0], "Variables: ", coverage[z][1] , "| Expected Values: ", coverage[z][2], "| Receive Values: ", coverage[z][3], "| Result: ", coverage[z][4], "| Error: ", coverage[z][5])
    print("===================================END========================================\n")

if __name__ == "__main__":
    
    testTPs(lTPs)