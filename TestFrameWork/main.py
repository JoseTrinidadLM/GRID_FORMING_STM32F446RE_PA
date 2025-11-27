import os
import subprocess
import sys
import time

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


def run_test_case(tc_name):
    tc_script = os.path.join(TC_FOLDER, f"{tc_name}.gdb")
    if not os.path.exists(tc_script):
        print(f"Test case script not found: {tc_script}")
        return

    cmd = [GDB_PATH, ELF_PATH, "-x", tc_script]
    print(f"Running: {' '.join(cmd)}")
    process = subprocess.run(cmd, capture_output=True, text=True)

    # Show output
    print(process.stdout)
    print(process.stderr)

    # Optional: Check for pass/fail keywords
    if "Hit main()" in process.stdout:
        print(f"{tc_name}: PASS")
    else:
        print(f"{tc_name}: FAIL")

if __name__ == "__main__":
    server = start_gdb_server()
    time.sleep(3)
    if len(sys.argv) < 2:
        print("Usage: python run_tc.py <TC_NAME>")
    else:
        run_test_case(sys.argv[1])
    server.terminate
