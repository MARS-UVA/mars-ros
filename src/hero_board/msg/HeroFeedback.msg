# Should correspond exactly to rpc-server/jetsonrpc.protofile
# Message documentation: https://wiki.ros.org/msg

uint8[] currents # uint8[] is the ros version of Python bytes()
float32 bucketLadderAngleL
float32 bucketLadderAngleR
bool depositBinRaised
bool bucketLadderLowered
