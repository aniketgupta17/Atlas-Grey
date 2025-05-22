# Project-CSSE4011

Advanced Embedded system project

## Communication Protocol

GATT protocol is used for the communication between different nodes and with some nodes possesing dual role as of
GATT server as well as Client.

GATT Workflow:

1. Server create a GATT service with characteristic ID
2. Server Initiates the BLE advertisement
3. Client scans all the packets, find the server by UUID
4. Client initiates the connection
5. Client sends GATT service discovery request to get a list of available services and characteristics
6. Client reads/writes or subscribes
