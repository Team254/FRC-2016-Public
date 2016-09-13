from shutil import copyfile
import subprocess
import os
import time

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
SAVE_LOC = os.path.join(os.path.expanduser("~"), "SmartDashboard", "save.xml")
FILES_DIR = os.path.join(SCRIPT_DIR, "boards")
BOARDS = [f for f in os.listdir(FILES_DIR) if os.path.isfile(os.path.join(FILES_DIR, f))]

BOARD_LOC_x = ["-1280", "0"]
BOARD_LOC_y = ["0", "0"]
BOARD_SIZE_x = ["1280", "1366"]
BOARD_SIZE_y = ["800", "500"]

print("Loaded " + str(len(BOARDS)) + " boards.")
count = 0
for b in BOARDS:
    print(" - " + b)
    print("   - " + os.path.join(FILES_DIR, b))
    copyfile(os.path.join(FILES_DIR, b), SAVE_LOC)
    subprocess.Popen(["java", "-jar", os.path.join(SCRIPT_DIR, "SmartDashboard.jar")], stdout=subprocess.DEVNULL)
    time.sleep(3)
    subprocess.Popen(["cmdow", "SmartDashboard - 254", "/REN", "254SMARTD" + str(count)])
    #, "/MOV",  BOARD_LOC[count], "/SIZ", BOARD_SIZE[count]
    time.sleep(1)
    subprocess.Popen(["cmdow", "254SMARTD" + str(count), "/MOV",  BOARD_LOC_x[count], BOARD_LOC_y[count], "/SIZ", BOARD_SIZE_x[count], BOARD_SIZE_y[count]])
    count = count + 1
