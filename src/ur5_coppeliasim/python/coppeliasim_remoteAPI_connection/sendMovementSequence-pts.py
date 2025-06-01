# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/movementViaRemoteApi.ttt
#
# Do not launch simulation, then run this script
#
# The client side (i.e. this script) depends on:
#
# sim.py, simConst.py, and the remote API library available
# in programming/remoteApiBindings/lib/lib
# Additionally you will need the python math and msgpack modules

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import math
import msgpack
import csv
from math import pi
import numpy as np
import os
import cv2

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    executedMovId='notReady'

    # targetArm='/blueArm'
    # targetArm='/redArm'
    targetArm     = '/UR5'

    stringSignalName=targetArm+'_executedMovId'

    def waitForMovementExecuted(id):
        global executedMovId
        global stringSignalName
        while executedMovId!=id:
            retCode,s=sim.simxGetStringSignal(clientID,stringSignalName,sim.simx_opmode_buffer)
            if retCode==sim.simx_return_ok:
                if type(s)==bytearray:
                    s=s.decode('ascii') # python2/python3 differences
                executedMovId=s

    # Start streaming stringSignalName string signal:
    sim.simxGetStringSignal(clientID,stringSignalName,sim.simx_opmode_streaming)
    
    # Get objects' handles
    res, prox_sens_handle = sim.simxGetObjectHandle( clientID, '/Proximity_sensor', sim.simx_opmode_blocking )
    res, vision_handle    = sim.simxGetObjectHandle( clientID, "/Vision_sensor", sim.simx_opmode_blocking )

    # Set-up some movement variables:
    # times = [0, 0.2, 0.4, 0.6, 0.8, 1, 1.2, 1.4, 1.6, 1.8, 2, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 3.6, 3.8, 4.0, 4.2, 4.4, 4.6, 4.8, 5.0, 5.2, 5.4, 5.6, 5.8, 6, 6.2, 6.4, 6.6, 6.8, 7, 7.2, 7.4, 7.6, 7.8, 8, 8.2, 8.4, 8.6, 8.8, 9, 9.2, 9.4, 9.6, 9.8, 10, 10.2, 10.4, 10.6, 10.8]
    times = np.linspace(0, 10, 54).tolist()
    j1    = []
    j2    = []
    j3    = []
    j4    = []
    j5    = []
    j6    = []

    print(os.chdir("/Users/carro/Documents/GITHUB/6-axis-Manipulator-Kinematics/src/ur5_coppeliasim/python/coppeliasim_connection/"))
    trajectory = []
    with open('trajectory.txt') as file_obj: 
        reader_obj = csv.reader(file_obj, quoting=csv.QUOTE_NONNUMERIC) 
        for row in reader_obj: 
            j1.append(row[0])
            j2.append(row[1])
            j3.append(row[2])
            j4.append(row[3])
            j5.append(row[4])
            j6.append(row[5])

    input("")

    # Start simulation:
    sim.simxStartSimulation( clientID, sim.simx_opmode_blocking )

    # Wait until ready:
    waitForMovementExecuted( 'ready' )

    # Home configuration
    q0 = [0.33, pi/6, -1.189, 2.127, 0.563, -2.138]

    # Send the movement for homing procedure:
    movementData       = {"id":"homing", "type":"pts", "times":[0, 0.05], "j1":[0, q0[0]], "j2":[0, q0[1]], "j3":[0, q0[2]], "j4":[0, q0[3]], "j5":[0, q0[4]], "j6":[0, q0[5]]}
    packedMovementData = msgpack.packb( movementData )
    
    sim.simxCallScriptFunction( clientID, targetArm, sim.sim_scripttype_childscript, 'legacyRemoteApi_movementDataFunction', [], [], [], 
                                packedMovementData, sim.simx_opmode_oneshot )

    # Execute movement sequence:
    sim.simxCallScriptFunction( clientID, targetArm, sim.sim_scripttype_childscript, 'legacyRemoteApi_executeMovement', [], [], [], 
                                'homing', sim.simx_opmode_oneshot )
    
    # Wait until above movement sequence finished executing:
    waitForMovementExecuted( 'homing' )

    input("Start trajectory")

    # Send the movement sequence:
    movementData       =  {"id":"movSeq1", "type":"pts", "times":times, "j1":j1, "j2":j2, "j3":j3, "j4":j4, "j5":j5, "j6":j6}
    packedMovementData = msgpack.packb( movementData )
    
    sim.simxCallScriptFunction( clientID, targetArm, sim.sim_scripttype_childscript, 'legacyRemoteApi_movementDataFunction', [], [], [], 
                                packedMovementData, sim.simx_opmode_oneshot )

    # Execute movement sequence:
    sim.simxCallScriptFunction( clientID, targetArm, sim.sim_scripttype_childscript, 'legacyRemoteApi_executeMovement', [], [], [], 
                                'movSeq1', sim.simx_opmode_oneshot )
    
    # Wait until above movement sequence finished executing:
    waitForMovementExecuted( 'movSeq1' )

    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor( clientID, prox_sens_handle, sim.simx_opmode_oneshot_wait )
    print(detectionState)
    while True:
        try:
            # returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor( clientID, prox_sens_handle, sim.simx_opmode_oneshot_wait )
            # print(detectionState)
            res, resolution, img = sim.simxGetVisionSensorImage( clientID, vision_handle, 0, sim.simx_opmode_oneshot_wait )
            cv2.imshow("img", img)
        except:
            print("error reading proximity sensor")
    
    input("Press enter to close connection...")

    sim.simxStopSimulation( clientID, sim.simx_opmode_blocking )
    sim.simxGetStringSignal( clientID, stringSignalName, sim.simx_opmode_discontinue )
    sim.simxGetPingTime( clientID )

    # Now close the connection to CoppeliaSim:
    sim.simxFinish( clientID )

else:
    print( 'Failed connecting to remote API server' )

print( 'Program ended' )

