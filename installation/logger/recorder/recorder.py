import json
import sys
import time
import uuid
import sqlite3

import paho.mqtt.client as mqtt

def onConnect(client, userData, flags, resultCode):
    print("Connected with result code "+str(resultCode))
    client.subscribe("/robot_logging")
    client.subscribe("/replay_log_request")

def onMessage(client, userData, msg):
    msgPayloadJson = json.loads(msg.payload)
    if msg.topic == '/robot_logging':
        saveLogMessage(userData, msgPayloadJson)
    elif msg.topic == '/replay_log_request':
        replayLogs(client, userData, msgPayloadJson)

def saveLogMessage(sqlSession, msgPayloadArray):
    print msgPayloadArray
    for jsonObject in msgPayloadArray:
        sqlSession.cursor.execute(
            "INSERT INTO logs (walltime, jsonPayload) VALUES (?, ?)",
            (jsonObject["walltime"], json.dumps(jsonObject)))
    sqlSession.connection.commit()

def replayLogs(client, sqlSession, msgPayloadObject):
    numLogsToReplay = int(msgPayloadObject["numlogs"])
    print numLogsToReplay
    replayMessageArray = []
    sqlSession.cursor.execute(
        "SELECT jsonPayload FROM logs ORDER BY walltime DESC LIMIT ?",
        (numLogsToReplay,))
    while True:
        row = sqlSession.cursor.fetchone()
        if row is None:
            break
        replayMessageArray.append(json.loads(row[0]))
    client.publish('/replay_logging', payload=json.dumps(replayMessageArray))

def onDisconnect(client, userData, resultCode):
    print "disconnected: ", resultCode

class SqlSession:
    def __init__(self, dbFilename):
        self.connection = sqlite3.connect(
            dbFilename,
            check_same_thread=False)
        self.cursor = self.connection.cursor()
        self.cursor.execute(
            "CREATE TABLE IF NOT EXISTS logs ("
            " walltime INTEGER,"
            " jsonPayload TEXT"
            ")")
        self.cursor.execute(
            "CREATE INDEX IF NOT EXISTS timeindex ON logs (walltime)")
        self.connection.commit()

def makeUserData(dbFile):
    return userData

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: $", sys.argv[0], "<mqtt-host> <database-file>"
        exit(-1)
    client = mqtt.Client(
        client_id="recorder-" + str(uuid.uuid1()),
        userdata=SqlSession(sys.argv[2]))
    client.on_connect = onConnect
    client.on_message = onMessage
    client.on_disconnect = onDisconnect
    client.connect(sys.argv[1], 1883, 10)
    client.loop_start()
    # allow myself to be killed by C-c
    while True:
        time.sleep(1)
    client.loop_stop()
