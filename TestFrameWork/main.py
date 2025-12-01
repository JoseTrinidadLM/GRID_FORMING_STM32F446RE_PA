import os
import subprocess
import sys
import time
import re

import protocol_test

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
    # Find the number associated with var_name in Auto-display section
    expr_pattern = rf"\d+:\s+y\s+/d\s+{re.escape(var_name)}"
    expr_match = re.search(expr_pattern, output)
    if not expr_match:
        return None

    # Extract the number (Num) before the colon
    num_match = re.search(r"(\d+):\s+y\s+/d\s+" + re.escape(var_name), expr_match.group(0))
    if not num_match:
        return None
    num = num_match.group(1)

    # Now find the value for $num
    value_pattern = rf"\${num}\s*=\s*(.+?)(?:\n|$)"
    value_match = re.search(value_pattern, output)
    if value_match:
        return value_match.group(1).strip()
    return None


def init_load():
    tc_script = os.path.join(TC_FOLDER, "init_load.gdb")
    if not os.path.exists(tc_script):
        print(f"Program load script not found {tc_script}")
        return
    
    cmd = [GDB_PATH, "-batch", ELF_PATH, "-x", tc_script]
    print(f"Loading Program: {' '.join(cmd)}")
    try:
        process = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        print("GDB execution timed out after 60 seconds")
        return

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

def run_multiple_scripts(scripts):
    server = start_gdb_server()
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
    server.terminate()
    server.wait()
    time.sleep(2)
    return output

def load_program():
    server = start_gdb_server()
    init_load()        #Load source code to board
    server.terminate()
    server.wait()
    time.sleep(2)

#Output #TP, Variables Names, Expected Values, Receive Values, Result, Error Message

def test_procedure(test_number, variables_name_list , expected_values_list, scripts_list):
    output = [test_number, variables_name_list, expected_values_list, None, None, None]
    print(f"\n====================================TP-{test_number}====================================\n")
    out = run_multiple_scripts(scripts_list)
    values = []
    for x in range(len(variables_name_list)):
        values.append(parse_gdb_value(out, variables_name_list[x]))
    output[3] = values
    output[4] = False

    for x in range(len(variables_name_list)):
        if output[3][x] == str(expected_values_list[x]):
            output[4] = True
        else:
            output[4] = False
            break
    return output

def tp001():
    output = test_procedure(1, ["systemState"], [0], ["init.gdb", "main_while.gdb", "data.gdb", "disconnect.gdb"])
    return output

def tp002():
    output = test_procedure(2, ["(LED.pGPIOx.ODR >> 5) & 0b1"], [0], ["init.gdb", "main_while.gdb","data.gdb", "disconnect.gdb"])
    return output

def tp004():
    output = test_procedure(4, ["systemState"], [0], ["init.gdb", "main_while.gdb", "button1.gdb", "button2.gdb", "button1.gdb", "data.gdb", "disconnect.gdb"])
    return output

def tp005():
    output = test_procedure(5, ["systemState"], [1], ["init.gdb", "main_while.gdb", "button1.gdb", "button2.gdb", "data.gdb", "disconnect.gdb"])
    return output

def tp008():
    output = test_procedure(8, ["systemState"], [1], ["init.gdb", "main_while.gdb", "button1.gdb", "button2.gdb", "button2.gdb", "data.gdb", "disconnect.gdb"])
    return output

def tp009():
    output = test_procedure(9, ["systemState", "(LED.pGPIOx.ODR >> 5) & 0b1"], [3, 0], ["init.gdb", "main_while.gdb", "button1.gdb", "button2.gdb", "heartbeat.gdb", "heartbeat.gdb", "data.gdb", "disconnect.gdb"])
    return output

def tp023():
    cmd_command('01')
    output = test_procedure(23, ["systemState"], [1], ["init.gdb", "heartbeat.gdb", "command.gdb", "main_while.gdb", "data.gdb", "disconnect.gdb"])
    return output

lTPs = [tp001, tp002, tp004, tp005, tp008, tp009, tp023]

def testTPs(ltps):
    nTPs = len(ltps)
    fTPs = 0
    coverage = []
    y = 0
    for x in ltps:
        coverage.append(x())
        if not coverage[y][4]:
            fTPs += 1
        y += 1
        time.sleep(10)
    print("\n\n==============================================================================")
    print("===============================COVERAGE RESULT================================")
    print("==============================================================================")
    print("Number of TPs: ", nTPs, "   Fail TPs: ", fTPs)
    for z in range(len(coverage)):
        print("#TP ",coverage[z][0], "Variables: ", coverage[z][1] , "| Expected Values: ", coverage[z][2], "| Receive Values: ", coverage[z][3], "| Result: ", coverage[z][4], "| Error: ", coverage[z][5])
    print("===================================END========================================\n")

if __name__ == "__main__":
    
    testTPs(lTPs)