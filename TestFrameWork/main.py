import os
import subprocess
import sys

#Paths
GDB_PATH = r"C:\path\to\arm-none-eabi-gdb.exe"
ELF_PATH = r"C:\Users\jtlopez\Documents\InternProject\STM32F446RE\GRID_FORMING\Debug\GRID_FORMING.elf"
TC_FOLDER = r"C:\Users\jtlopez\Documents\InternProject\debug_tests"


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
    if len(sys.argv) < 2:
        print("Usage: python run_tc.py <TC_NAME>")
    else:
        run_test_case(sys.argv[1])
