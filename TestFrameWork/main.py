import os
import subprocess
import sys
import time
import re

#Paths
GDB_SERVER = r"C:\ST\STM32CubeCLT_1.20.0\STLink-gdb-server\bin\ST-LINK_gdbserver.exe"
GDB_PATH = r"C:\ST\STM32CubeCLT_1.20.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe"
ELF_PATH = r"C:\Users\jtlopez\Documents\InternProject\STM32F446RE\GRID_FORMING\Debug\GRID_FORMING.elf"
TC_FOLDER = r"C:\Users\jtlopez\Documents\InternProject\STM32F446RE\TestFrameWork"



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

    # Pattern 1: Display format "$N = value"
    pattern1 = rf"\$\d+\s*=\s*(.+?)(?:\n|$)"
    match = re.search(pattern1, output)
    if match:
        return match.group(1).strip()
    
    # Pattern 2: Print format "variable = value"
    pattern2 = rf"{var_name}\s*=\s*(.+?)(?:\n|$)"
    match = re.search(pattern2, output)
    if match:
        return match.group(1).strip()

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

def tp001():
    print("\n====================================TP-001====================================\n")
    server = start_gdb_server()
    output = run_multiple_scripts(["init.gdb", "main_while.gdb", "data.gdb"])
    server.terminate()
    server.wait()
    time.sleep(2)
    systemState = parse_gdb_value(output, "systemState")
    print("Value: "+systemState)
    if systemState == "0":
        return True
    return False

def tp002():
    print("\n====================================TP-002====================================\n")
    server = start_gdb_server()
    output = run_multiple_scripts(["init.gdb", "main_while.gdb","data.gdb"])
    server.terminate()
    server.wait()
    time.sleep(2)
    ledState = parse_gdb_value(output, "(LED.pGPIOx.ODR >> 5) & 0b1")
    print("Value: "+ledState)
    if ledState == "0":
        return True
    return False

def tp004():
    print("\n====================================TP-004====================================\n")
    server = start_gdb_server()
    output = run_multiple_scripts(["init.gdb", "main_while.gdb", "button1.gdb", "button2.gdb", "button1.gdb", "data.gdb"])
    server.terminate()
    server.wait()
    time.sleep(2)
    systemState = parse_gdb_value(output, "systemState")
    print("\nValue: "+systemState+"\n")
    if systemState == "0":
        return True
    return False

def tp005():
    print("\n====================================TP-005====================================\n")
    server = start_gdb_server()
    output = run_multiple_scripts(["init.gdb", "main_while.gdb", "button1.gdb", "button2.gdb", "data.gdb"])
    server.terminate()
    server.wait()
    time.sleep(2)
    systemState = parse_gdb_value(output, "systemState")
    print("\nValue: "+systemState+"\n")
    if systemState == "1":
        return True
    return False


if __name__ == "__main__":
    #load_program()           #Neccessary only if there are changes to source code
    
    print(tp001())