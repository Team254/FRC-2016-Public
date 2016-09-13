import sys
import time
import logging
import json

sys.path.append("pynetworktables-2015.3.2-py2.7.egg")
from networktables import NetworkTable

logging.basicConfig(level=logging.DEBUG)
NetworkTable.initialize()

table = NetworkTable.getTable("SmartDashboard")

def valueChanged(table, key, value, isNew):
    print("Value Changed", table.path, key, value)
table.addTableListener(valueChanged)

i = 0
while True:
    table.putNumber("server_count", i)
    table.putBoolean("have_ball", i % 2 == 0)
    if i % 5 == 0:
        table.putString(
            "auto_options",
            json.dumps([
                "some auto option",
                "another auto option",
                "crappy option",
                "4th option"]))
    table.putNumber("Air Pressure psi", i % 120)
    i += 1
    time.sleep(1)
