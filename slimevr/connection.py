#!/usr/bin/python

import socket
import struct
import time
from slimevr.constants import *
from slimevr.logger import Logger
from slimevr.sensor import Sensor, SENSOR_OFFLINE, SENSOR_OK, SENSOR_ERROR
from slimevr.quat import Quaternion

class Connection:
    def __init__(self):
        self.l = None
        self.m_PacketNumber = 0
        self.sock = None
        self.m_ServerHost = '<broadcast>'
        self.m_ServerPort = 6969
        self.m_Connected = False
        self.m_Logger = Logger()
        self.m_LastConnectionAttemptTimestamp = 0
        self.m_LastPacketTimestamp = 0
        self.m_AckedSensorState = list()
        self.m_LastSensorInfoPacketTimestamp = 0

        for i in range(0, MAX_IMU_COUNT):
            self.m_AckedSensorState.append(SENSOR_OFFLINE)

        self.sensors = [ Sensor("python-slimevr00", IMU_UNKNOWN, 0, 0, 0.0) ]

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setblocking(0)
        self.sock.bind(('', 0))


    def beginPacket(self):
        self.l = list()

    def endPacket(self):
        self.sock.sendto(bytes().join(self.l), (self.m_ServerHost, self.m_ServerPort))

    def sendFloat(self, val):
        self.l.append(struct.pack('!f', val))

    def sendByte(self, val):
        self.l.append(struct.pack('!B', val))

    def sendShort(self, val):
        self.l.append(struct.pack('!H', val))

    def sendInt(self, val):
        self.l.append(struct.pack('!I', val))

    def sendLong(self, val):
        self.l.append(struct.pack('!Q', val))

    def sendBytes(self, val):
        self.l.append(struct.pack('!{}s'.format(len(val)), val))

    def sendPacketNumber(self):
        pn = self.m_PacketNumber
        self.m_PacketNumber += 1
        self.sendLong(pn)

    def sendShortString(self, val):
        size = len(val)
        self.sendByte(size)
        self.sendBytes(val)

    def sendPacketType(self, val):
        self.sendByte(0)
        self.sendByte(0)
        self.sendByte(0)
        self.sendByte(val)

    def sendLongString(self, val):
        size = len(val)
        self.sendInt(size)
        self.sendBytes(val)

    # PACKET_HEARTBEAT 0
    def sendHeartbeat(self):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_HEARTBEAT)
        self.sendPacketNumber()
        self.endPacket()

    # PACKET_ACCEL 4
    def sendSensorAcceleration(self, sensorId, vector):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_ACCEL)
        self.sendPacketNumber()
        self.sendFloat(vector[0])
        self.sendFloat(vector[1])
        self.sendFloat(vector[2])
        self.sendByte(sensorId)
        self.endPacket()

    # PACKET_BATTERY_LEVEL 12
    def sendSensorAcceleration(self, batteryVoltage, batteryPercentage):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_ACCEL)
        self.sendPacketNumber()
        self.sendFloat(batteryVoltage)
        self.sendFloat(batteryPercentage)
        self.endPacket()

    # PACKET_TAP 13
    def sendSensorTap(self, sensorId, value):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_TAP)
        self.sendPacketNumber()
        self.sendByte(sensorId)
        self.sendByte(value)
        self.endPacket()

    # PACKET_ERROR 14
    def sendSensorError(self, sensorId, error):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_ERROR)
        self.sendPacketNumber()
        self.sendByte(sensorId)
        self.sendByte(error)
        self.endPacket()

    # PACKET_SENSOR_INFO 15
    def sendSensorInfo(self, sensor):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_SENSOR_INFO)
        self.sendPacketNumber()
        self.sendByte(sensor.getSensorId())
        self.sendByte(sensor.getSensorState())
        self.sendByte(sensor.getSensorType())
        self.endPacket()

    # PACKET_ROTATION_DATA 17
    def sendRotationData(self, sensorId, quaternion, dataType, accuracyInfo):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_ROTATION_DATA)
        self.sendPacketNumber()
        self.sendByte(sensorId)
        self.sendByte(dataType)
        self.sendFloat(quaternion.x)
        self.sendFloat(quaternion.y)
        self.sendFloat(quaternion.z)
        self.sendFloat(quaternion.w)
        self.sendByte(accuracyInfo)
        self.endPacket()

    # PACKET_MAGNETOMETER_ACCURACY 18
    def sendMagnetometerAccuracy(self, sensorId, accuracyInfo):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_MAGNETOMETER_ACCURACY)
        self.sendPacketNumber()
        self.sendByte(sensorId)
        self.sendByte(accuracyInfo)
        self.endPacket()

    # PACKET_SIGNAL_STRENGTH 19
    def sendSignalStrength(self, signalStrength):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_SIGNAL_STRENGTH)
        self.sendPacketNumber()
        self.sendByte(255)
        self.sendByte(signalStrength)
        self.endPacket()

    # PACKET_TEMPERATURE 20
    def sendTemperature(self, sensorId, temperature):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_TEMPERATURE)
        self.sendPacketNumber()
        self.sendByte(sensorId)
        self.sendFloat(temperature)
        self.endPacket()

    # PACKET_FEATURE_FLAGS 22
    def sendFeatureFlags(self):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_FEATURE_FLAGS)
        self.sendPacketNumber()
        self.endPacket()

    def sendTrackerDiscovery(self):
        if self.m_Connected: return
        self.beginPacket()
        self.sendPacketType(PACKET_HANDSHAKE)
        self.sendLong(0) # Packet number is always 0 for handshake
        self.sendInt(BOARD_UNKNOWN)
        # This is kept for backwards compatibility,
        # but the latest SlimeVR server will not initialize trackers
        # with firmware build > 8 until it recieves a sensor info packet
        self.sendInt(IMU_UNKNOWN)
        self.sendInt(MCU_UNKNOWN)
        self.sendInt(0) # IMUInfo[0]
        self.sendInt(0) # IMUInfo[1]
        self.sendInt(0) # IMUInfo[2]
        self.sendInt(FIRMWARE_BUILD_NUMBER)
        self.sendShortString(FIRMWARE_VERSION)
        self.sendBytes(MAC)
        self.endPacket()

    def returnLastPacket(self):
        if not self.m_Connected: return
        self.beginPacket()
        self.sendBytes(self.m_Packet)
        self.endPacket()

    def searchForServer(self):
        while True:
            try:
                self.m_Packet, address = self.sock.recvfrom(4096)
            except socket.error:
                break
            if self.m_Packet[0] == PACKET_HANDSHAKE:
                if self.m_Packet[1:13] != b'Hey OVR =D 5':
                    self.m_Logger.error('Received invalid handshake packet')
                    continue
                self.m_ServerHost = address[0]
                self.m_ServerPort = address[1]
                self.m_LastPacketTimestamp = millis()
                self.m_Connected = True
                self.m_Logger.debug('Handshake successful, server is {}:{}'.format(self.m_ServerHost, self.m_ServerPort))
        now = millis()
        if self.m_LastConnectionAttemptTimestamp + 1000 < now:
            self.m_LastConnectionAttemptTimestamp = now
            self.m_Logger.info('Searching for the server on the local network...')
            self.sendTrackerDiscovery()

    def updateSensorState(self):
        if millis() - self.m_LastSensorInfoPacketTimestamp <= 1000:
            return
        self.m_LastSensorInfoPacketTimestamp = millis()
        for i in range(0, MAX_IMU_COUNT):
            if self.m_AckedSensorState[i] != self.sensors[i].getSensorState():
                self.sendSensorInfo(self.sensors[i])

    def reset(self):
        self.m_PacketNumber = 0
        self.m_Connected = False
        self.m_ServerHost = '<broadcast>'
        for i in range(0, MAX_IMU_COUNT):
            self.m_AckedSensorState[i] = SENSOR_OFFLINE
        self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setblocking(0)
        self.sock.bind(('', 0))


    def update(self):
        self.updateSensorState()

        if not self.m_Connected:
            self.searchForServer()
            return
        if self.m_LastPacketTimestamp + TIMEOUT <= millis():
            self.reset()
            self.m_Logger.warn('Connection to server timed out')
            return
        try:
            self.m_Packet, self.address = self.sock.recvfrom(4096)
        except socket.error:
            return
        self.m_LastPacketTimestamp = millis();
        packet_type = struct.unpack('!I', self.m_Packet[0:4])[0]
        if packet_type == PACKET_RECEIVE_HEARTBEAT:
            self.sendHeartbeat()
        elif packet_type == PACKET_RECEIVE_VIBRATE:
            pass
        elif packet_type == PACKET_RECEIVE_HANDSHAKE:
            self.m_Logger.warn('Handshake received again, ignoring')
        elif packet_type == PACKET_RECEIVE_COMMAND:
            pass
        elif packet_type == PACKET_CONFIG:
            pass
        elif packet_type == PACKET_PING_PONG:
            self.returnLastPacket()
        elif packet_type == PACKET_SENSOR_INFO:
            if len(self.m_Packet) < 6:
                self.m_Logger.warn("Wrong sensor info packet")
            else:
                for i in range(0, MAX_IMU_COUNT):
                    if self.m_Packet[4] == self.sensors[i].getSensorId():
                        self.m_AckedSensorState[i] = self.m_Packet[5]
                        break
        elif packet_type == PACKET_FEATURE_FLAGS:
            # TODO
            pass
        else:
            self.m_Logger.warn('Received unknown packet of type: {}'.format(packet_type))

def millis():
    return time.time_ns() // 1_000_000

if __name__ == '__main__':
    angle = 0.0

    connection = Connection()

    while True:
        time.sleep(0.01)
        connection.update()
        connection.sendRotationData(0, Quaternion.fromAxisAngle(1.0, 1.0, 1.0, angle), DATA_TYPE_NORMAL, 0)
        angle += 0.5
