/**
    DSSN Final Project
    Base station code
    TODO Does this parse and send startup responses and data responses from other nodes to MATLAB correctly
*/

// Uncomment this define to use this software with the nrf24l01 radios
#define USE_RF24_RADIO
// Uncomment to compile serial debug statements
#define SERIAL_DEBUG

#ifdef USE_RF24_RADIO
#include "RF24.h"
#include <SPI.h>
#endif

#ifdef USE_RF24_RADIO
// Instantiate radio object
RF24 radio(8, 7);
#endif

// -------------- DEFINES --------------
// Unique node identifier
// NOTE: Start incrementing the node id from 1, so 0 represents no node
#define NODE_ID 8

#define MATLAB_NODE_ID 9

// Maximum number of nodes in a message path
#define MAX_NODE_PATH 9

// Maximum possible number of neighbor nodes
#define MAX_NUM_NEIGHBORS 9

// Empty message payload
#define EMPTY_MSG_PAYLOAD 0

// Length of the application communication buffer is equal to the maximum message length of 28 bytes
//#define COMM_BUFFER_SIZE 28U

// Main Channel ID
#define MAIN_CHANNEL 76

// ACK Channel ID
#define ACK_CHANNEL 80

// Declare channel ID
const uint64_t pipe = 0xE8E8F0F0E1LL;

// Declare message properties
#define MSG_NODE_ID_OFFSET 4
#define MSG_NODE_ID_MASK 0xF0
#define MSG_TYPE_MASK 0x0F

#define UPPER_BYTE_OFFSET 8
#define UPPER_BYTE_MASK 0xFF00

// -------------- STRUCTS & ENUMS --------------
// Message packet structure
typedef struct __attribute__((__packed__))
{
  uint8_t header = 0; // First (Most Significant) 4 bits = Node ID, Second 4 bits = Message type
  uint8_t* payload; // Add the payload as a byte stream
} message_S;

// I love my packed structs, this contains the list of reachable neighbor ids and power levels
typedef struct __attribute__((__packed__))
{
  uint8_t node_ids[MAX_NUM_NEIGHBORS]; // These two arrays are alligned by id and cost
  uint8_t edge_costs[MAX_NUM_NEIGHBORS]; // Edge costs represent power required to reach the node
} neighbor_list_S;

// Message Payload Structs

// NEIGHBOR_QUERY Payload
typedef struct __attribute__((__packed__))
{
  uint8_t broadcast_power = 0;
} neighbor_query_payload_S;

// NEIGHBOR_RSP Payload
typedef struct __attribute__((__packed__))
{
  uint8_t querying_node_id = 0;
  uint8_t received_power = 0;
} neighbor_rsp_payload_S;

// NEIGHBOR_RSP_ACK Payload
typedef struct __attribute__((__packed__))
{
  uint8_t node_acknowledged = 0;
} neighbor_rsp_ack_payload_S;

// STARTUP_MSG Payload
typedef struct __attribute__((__packed__))
{
  uint8_t target_node;
  uint8_t node_path[MAX_NODE_PATH];
} startup_msg_payload_S;

// STARTUP_RSP Payload
typedef struct __attribute__((__packed__))
{
  uint8_t target_node;
  uint8_t node_path[MAX_NODE_PATH];
  uint8_t node_ids[MAX_NUM_NEIGHBORS]; // These two arrays are alligned by id and cost
  uint8_t edge_costs[MAX_NUM_NEIGHBORS];
} startup_rsp_payload_S;

// DATA_QUERY Payload
typedef struct __attribute__((__packed__))
{
  uint8_t target_node;
  uint8_t node_path[MAX_NODE_PATH];
  uint8_t request; // TODO Create enum of request numbers
} data_query_payload_S;

// DATA_RSP Payload
typedef struct __attribute__((__packed__))
{
  uint8_t target_node;
  uint8_t node_path[MAX_NODE_PATH];
  uint8_t data; // TODO Create enum of request numbers
} data_rsp_payload_S;

// DATA_RSP_ACK Payload
typedef struct __attribute__((__packed__))
{
  uint8_t target_node;
} data_rsp_ack_payload_S;

// ERROR_MSG Payload
typedef struct __attribute__((__packed__))
{
  uint8_t unreachable_node;
} error_msg_payload_S;

// Enumeration of possible node states
typedef enum
{
  WAIT_FOR_TOKEN,
  STARTUP_MODE,
  NORMAL_MODE
} states_E;

// Message IDs
typedef enum
{
  NEIGHBOR_QUERY = 0, // "Hello!"
  STARTUP_MSG = 1, // Begin startup
  STARTUP_RSP = 2, // Startup complete
  DATA_QUERY = 3, // Tell me some info
  DATA_RSP = 4, // Info received
  ERROR_MSG = 5, // Error message
  NEIGHBOR_RSP = 6, // Hello I heard you
  NEIGHBOR_RSP_ACK = 7, // I got your response, no need to resend
  TYPE_NONE = 8
} message_id_E;
// Error handling back to basestation

//union MsgPayload // Declare a simple union type
//{
//  neighbor_query_payload_S* nQueryPayload;
//  neighbor_rsp_payload_S* nRspPayload;
//  neighbor_rsp_ack_payload_S* nRspPayloadAck;
//  startup_msg_payload_S* startupMsgPayload;
//  startup_rsp_payload_S* startupRspPayload;
//  data_query_payload_S* dataQueryPayload;
//  data_rsp_payload_S* dataRspPayload;
//  data_rsp_ack_payload_S* dataRspAckPayload;
//  error_msg_payload_S* errorMsgPayload;
//};

typedef struct
{
  neighbor_query_payload_S nQueryPayload;
  neighbor_rsp_payload_S nRspPayload;
  neighbor_rsp_ack_payload_S nRspPayloadAck;
  startup_msg_payload_S startupMsgPayload;
  startup_rsp_payload_S startupRspPayload;
  data_query_payload_S dataQueryPayload;
  data_rsp_payload_S dataRspPayload;
  data_rsp_ack_payload_S dataRspAckPayload;
  error_msg_payload_S errorMsgPayload;
} MsgPayload;

typedef enum
{
  INFINITY_POWER = 0,
  MIN_POWER = 1,
  LOW_POWER = 2,
  HIGH_POWER = 3,
  MAX_POWER = 4
} edge_costs_E;

// -------------- GLOBAL VARIABLES --------------
// Initialize the node id queue
uint8_t payloadDataBuffer[32]; // Max serial buffer data size
uint8_t receiveBuffer[32]; // matlab interface
uint8_t transmitBuffer[64];// matlab interface
uint8_t nodeTransmitBuffer[32];

// -------------- FUNCTIONS --------------
// Function Prototypes
static void debugMessage(message_S* msgToDebug);

// Function to pack information into an empty message
void buildMessage(message_S* msg, uint8_t node_id, uint8_t message_type, uint8_t* payloadPtr)
{
  // Build the header with the node id and message type
  msg->header = ((node_id << MSG_NODE_ID_OFFSET) & MSG_NODE_ID_MASK) | (message_type & MSG_TYPE_MASK);
  // Build the message payload, more to come here...?
  msg->payload = payloadPtr;
}

// Function to parse message id
message_id_E getMessageIdFromHeader(uint8_t msgHeader)
{
  return (message_id_E)(msgHeader & MSG_TYPE_MASK);
}

// Function to parse node id
uint8_t getNodeIdFromHeader(uint8_t msgHeader)
{
  return (uint8_t)((msgHeader & MSG_NODE_ID_MASK) >> MSG_NODE_ID_OFFSET);
}

// Helper function to calculate the number of bytes to expect in a message payload
uint8_t getPayloadSize(message_id_E msgType)
{
  uint8_t numPayloadBytes = 0;
  switch (msgType)
  {
    case NEIGHBOR_QUERY:
      numPayloadBytes = sizeof(neighbor_query_payload_S);
      break;

    case NEIGHBOR_RSP:
      numPayloadBytes = sizeof(neighbor_rsp_payload_S);
      break;

    case NEIGHBOR_RSP_ACK:
      numPayloadBytes = sizeof(neighbor_rsp_ack_payload_S);
      break;

    case STARTUP_MSG:
      numPayloadBytes = sizeof(startup_msg_payload_S);
      break;

    case STARTUP_RSP:
      numPayloadBytes = sizeof(startup_rsp_payload_S);
      break;

    case ERROR_MSG:
      numPayloadBytes = sizeof(error_msg_payload_S);
      break;

    default:
#ifdef SERIAL_DEBUG
      Serial.println(F("[getPayloadSize] ERROR: Unknown message type!"));
#endif
      break;
  }
  return numPayloadBytes;
}

// Function to check the radio for ack
bool readMessage(message_S* currMessage) // TODO Read bytes from the radio over SPI
{
#ifdef USE_RF24_RADIO
  uint8_t numBytesInBuffer = radio.available();
#else
  //  uint8_t numBytesInBuffer = Serial.available();
#endif

  // If there are bytes in the buffer
  if (numBytesInBuffer != 0)
  {
    // Read in the payload
    if (radio.available())
    {
      if (radio.getDynamicPayloadSize() < 1)
      {
        Serial.println("Corrupt payload data has been flushed");
        return false;
      }
      radio.read(&payloadDataBuffer, sizeof(payloadDataBuffer));
      currMessage->header = payloadDataBuffer[0];
      currMessage->payload = (uint8_t*) & (payloadDataBuffer[1]);
      return true;
    }
    else
    {
      currMessage = NULL;
      return false;
    }
  }
}

// Debug function to display message contents
static void debugMessage(message_S * msgToDebug)
{
#ifdef SERIAL_DEBUG
  MsgPayload debugMsgPayloads;
  message_id_E debugMsgType;
  uint8_t debugMsgNodeId = 0;

  parseHeader(msgToDebug, &debugMsgType, &debugMsgNodeId);
  Serial.print(F("Sent from Node "));
  Serial.println(debugMsgNodeId, DEC);

  parsePayload(msgToDebug, &debugMsgPayloads);

  Serial.print(F("Message type: "));
  switch (debugMsgType)
  {
    case NEIGHBOR_QUERY:
      Serial.println(F("NEIGHBOR_QUERY"));
      Serial.print(F("broadcast_power = "));
      Serial.println(debugMsgPayloads.nQueryPayload.broadcast_power, DEC);
      break;

    case NEIGHBOR_RSP:
      Serial.println(F("NEIGHBOR_RSP"));
      Serial.print(F("querying_node_id = "));
      Serial.println(debugMsgPayloads.nRspPayload.querying_node_id, DEC);
      Serial.print(F("received_power = "));
      Serial.println(debugMsgPayloads.nRspPayload.received_power, DEC);
      break;

    case NEIGHBOR_RSP_ACK:
      Serial.println(F("NEIGHBOR_RSP_ACK"));
      Serial.print(F("node_acknowledged = "));
      Serial.println(debugMsgPayloads.nRspPayloadAck.node_acknowledged, DEC);
      break;

    case STARTUP_MSG:
      Serial.println(F("STARTUP_MSG"));
      Serial.print(F("target_node = "));
      Serial.println(debugMsgPayloads.startupMsgPayload.target_node, DEC);
      Serial.print(F("node_path = ["));
      for (uint8_t idx = 0; idx < MAX_NODE_PATH; ++idx)
      {
        Serial.print(debugMsgPayloads.startupMsgPayload.node_path[idx], DEC);
        ((idx == MAX_NODE_PATH - 1) ? Serial.print(F("]\n\r")) : Serial.print(F(", ")));
      }
      break;

    case STARTUP_RSP:
      Serial.println(F("STARTUP_RSP"));
      Serial.print(F("target_node = "));
      Serial.println(debugMsgPayloads.startupRspPayload.target_node, DEC);
      Serial.print(F("node_path = ["));
      for (uint8_t idx = 0; idx < MAX_NODE_PATH; ++idx)
      {
        Serial.print(debugMsgPayloads.startupRspPayload.node_path[idx], DEC);
        ((idx == MAX_NODE_PATH - 1) ? Serial.print(F("]\n\r")) : Serial.print(F(", ")));
      }

      Serial.println(F("neighbor_list = (node_id,edge_cost) : "));
      for (uint8_t idx = 0; idx < MAX_NODE_PATH; ++idx)
      {
        Serial.print(F("("));
        Serial.print(debugMsgPayloads.startupRspPayload.node_ids[idx], DEC);
        Serial.print(F(","));
        Serial.print(debugMsgPayloads.startupRspPayload.edge_costs[idx], DEC);
        ((idx == MAX_NODE_PATH - 1) ? Serial.println(F(")")) : Serial.print(F(")")));
      }
      break;

    case DATA_QUERY:
      Serial.println(F("DATA_QUERY"));
      // TODO Add func as needed debugMsgPayloads.dataQueryPayload
      break;

    case DATA_RSP:
      Serial.println(F("DATA_RSP"));
      // TODO Add func as needed debugMsgPayloads.dataRspPayload
      break;

    case ERROR_MSG:
      Serial.println(F("ERROR_MSG"));
      Serial.print(F("Unreachable node: "));
      Serial.println(debugMsgPayloads.errorMsgPayload.unreachable_node);
      break;

    case TYPE_NONE:
      Serial.println(F("TYPE_NONE"));
      break;

    default:
      Serial.println(F("Unknown"));
      break;
  }
#endif
}

// Function to parse a received message header TODO rename parse header
void parseHeader(message_S * receivedMsg, message_id_E * messageType, uint8_t* txNodeId)
{
  // Mask out the message type
  *messageType = getMessageIdFromHeader(receivedMsg->header);
  *txNodeId = getNodeIdFromHeader(receivedMsg->header);
}

// Return true when the specified ack has been received, otherwise rebroadcast the message
bool waitForAck(message_S * messageToSend, message_id_E ackToReceive, uint8_t numRetries, uint32_t timeBetweenRetries)
{
  uint8_t numRetriesSent = 0;
  uint16_t randTime = random(800); // Wait a random amount of time 0-.25sec
  uint32_t startTimeStamp = 0; // Timer to indicate when to retry the message
  delay(randTime);
#ifdef SERIAL_DEBUG
  Serial.println(F("Broadcasting intended message..."));
#endif

  // Transmit the message
  sendMessage(messageToSend, false); // TODO Replace with radio interface

  startTimeStamp = millis(); // Start the timer

  // Check the radio for an incoming ack
  //while(!checkForMessageType(ackToReceive)) // TODO what if there is garbage in the buffer?, need to flush it
  while (!radio.available())
  {
    // If timer has elapsed, retry the message
    if (((uint32_t)(millis() - startTimeStamp)) > timeBetweenRetries)  // Wait interval in between message retries, if no ack
    {
      // If we have retried the message too many times
      if (numRetriesSent >= numRetries)
      {
        // Failure
        return false;
      }
      else // Try again
      {
#ifdef SERIAL_DEBUG
        Serial.println(F("[No Ack] Broadcasting intended message again..."));
#endif
        ++numRetriesSent; // Record the retry
        sendMessage(messageToSend, false); // Send the message again
        startTimeStamp = millis(); // Restart the timer
      }
    }
  }

  // If the expected ack is received
  return true;
}

// Wrapper function that handles message sizing when sending a message
// NOTE: Different for basestation node
void sendMessage(message_S * msgToSend, bool sendToMatlab)
{
  // Calculate size of the respective payload in bytes
  uint8_t numBytesInPayload = getPayloadSize(getMessageIdFromHeader(msgToSend->header));

  // Send this message to the basestation
  if (sendToMatlab)
  {
    startup_rsp_payload_S* sRspPayload;
    data_rsp_payload_S* dRspPayload;
    error_msg_payload_S* errorMsgPayload;
    
    switch (getMessageIdFromHeader(msgToSend->header))
    {
      case STARTUP_RSP:
#ifdef SERIAL_DEBUG
        Serial.println(F("Transmitting STARTUP_RSP to MATLAB"));
#endif  
        debugMessage(msgToSend);
        sRspPayload = (startup_rsp_payload_S*) msgToSend->payload;
//        fixList((uint8_t*)&(sRspPayload->node_path[0]));
        transmitBuffer[0] = '2'; // Startup response message type
        transmitBuffer[1] = ',';
        if (sRspPayload->node_path[0] == 0)
        {
          transmitBuffer[2] = (NODE_ID + '0');
        }
        else
        {
          transmitBuffer[2] = (sRspPayload->node_path[0] + '0');
        }
        

        // Neighbor list and pointer
        for (uint8_t idx = 0; idx < (MAX_NUM_NEIGHBORS * 4); idx += 4)
        {
          transmitBuffer[idx + 3] = ',';
          transmitBuffer[idx + 4] = (sRspPayload->node_ids[idx / 4] + '0');
          transmitBuffer[idx + 5] = '_';
          transmitBuffer[idx + 6] = (sRspPayload->edge_costs[idx / 4] + '0');
        }
        transmitBuffer[39] = '\n'; // Add end of message char
        break;

      case DATA_RSP:
        dRspPayload = (data_rsp_payload_S*) msgToSend->payload;
        transmitBuffer[0] = '4'; // Data response message type
        transmitBuffer[1] = ',';
        transmitBuffer[2] = (dRspPayload->node_path[0] + '0');
        transmitBuffer[3] = ',';
        transmitBuffer[4] = dRspPayload->data; // Data
        transmitBuffer[5] = '\n';
        break;

      case ERROR_MSG:
        errorMsgPayload = (error_msg_payload_S*) msgToSend->payload;
        transmitBuffer[0] = '5'; // Error message type
        transmitBuffer[1] = ',';
        transmitBuffer[2] = (errorMsgPayload->unreachable_node + '0');
        transmitBuffer[3] = '\n';

      default:
        // Error case
        break;
    }
    // Send the message with the appropriate number of bytes
    Serial.write((uint8_t*)&transmitBuffer, sizeof(transmitBuffer));
  }
  else
  {
    //    Serial.print("Sending... ");
    //    Serial.println((sizeof(uint8_t) + numBytesInPayload));
    //    uint8_t msg[3] = {128,2,3};
    //    debugMessage(msgToSend);
    //    Serial.print("\n");

    // Buffer the outgoing data
    memcpy((uint8_t*) & (nodeTransmitBuffer[0]), (uint8_t*) & (msgToSend->header), sizeof(uint8_t));
    memcpy((uint8_t*) & (nodeTransmitBuffer[1]), (uint8_t*)msgToSend->payload, numBytesInPayload);

#ifdef USE_RF24_RADIO
    radio.stopListening();
    radio.write((uint8_t*) & (nodeTransmitBuffer[0]), (sizeof(uint8_t) + numBytesInPayload));
    radio.startListening();
#else
    // Send the message with the appropriate number of bytes
    //    Serial.write((uint8_t*)msgToSend, (sizeof(uint8_t) + numBytesInPayload));
#endif
  }
}

// Parse the payload of the received message into the intended message struct
void parsePayload(message_S * receivedMsg, MsgPayload * receivedMsgPayloads)
{ // We're transforming the byte stream into a known data structure that we can easily access
  // Check the message type and cast the payload as appropriate payload struct
  switch (getMessageIdFromHeader(receivedMsg->header))
  {
    case NEIGHBOR_QUERY:
      memcpy(&(receivedMsgPayloads->nQueryPayload), (neighbor_query_payload_S*)(receivedMsg->payload), sizeof(neighbor_query_payload_S));
      break;

    case NEIGHBOR_RSP:
      memcpy(&(receivedMsgPayloads->nRspPayload), (neighbor_rsp_payload_S*)(receivedMsg->payload), sizeof(neighbor_rsp_payload_S));
      //      receivedMsgPayloads->nRspPayload = (neighbor_rsp_payload_S*)(receivedMsg->payload);
      break;

    case NEIGHBOR_RSP_ACK:
      memcpy(&(receivedMsgPayloads->nRspPayloadAck), (neighbor_rsp_ack_payload_S*)(receivedMsg->payload), sizeof(neighbor_rsp_ack_payload_S));
      //      receivedMsgPayloads->nRspPayloadAck = (neighbor_rsp_ack_payload_S*)(receivedMsg->payload);
      break;

    case STARTUP_MSG:
      memcpy(&(receivedMsgPayloads->startupMsgPayload), (startup_msg_payload_S*)(receivedMsg->payload), sizeof(startup_msg_payload_S));
      break;

    case STARTUP_RSP:
      memcpy(&(receivedMsgPayloads->startupRspPayload), (startup_rsp_payload_S*)(receivedMsg->payload), sizeof(startup_rsp_payload_S));
      break;

    case DATA_QUERY:
      memcpy(&(receivedMsgPayloads->dataQueryPayload), (data_query_payload_S*)(receivedMsg->payload), sizeof(data_query_payload_S));
      //      receivedMsgPayloads->dataQueryPayload = (data_query_payload_S*)(receivedMsg->payload);
      break;

    case DATA_RSP:
      memcpy(&(receivedMsgPayloads->dataRspPayload), (data_rsp_payload_S*)(receivedMsg->payload), sizeof(data_rsp_payload_S));
      //      receivedMsgPayloads->dataRspPayload = (data_rsp_payload_S*)(receivedMsg->payload);
      break;

    case ERROR_MSG:
      memcpy(&(receivedMsgPayloads->errorMsgPayload), (error_msg_payload_S*)(receivedMsg->payload), sizeof(error_msg_payload_S));
      //      receivedMsgPayloads->errorMsgPayload = (error_msg_payload_S*)(receivedMsg->payload);
      break;

    default:
      // Error unrecognized message id
#ifdef SERIAL_DEBUG
      Serial.println("ERROR: Unknown message ID!");
#endif
      break;
  }
}

// Helper function to reverse the node list
void reverseNodeList(uint8_t* origNodeList, uint8_t* reversedNodeList, uint8_t listSize)
{
  // Grab the end of the list
  uint8_t* tailPtr = (origNodeList + listSize - 1);

  // Write the end of the list into the front of the list
  for (uint8_t i = 0; i < listSize; ++i)
  {
    // Ensure that zero padding is only at the end
    if (*(tailPtr - i) == 0)
    {
      *(reversedNodeList + (listSize - i - 1)) = 0;
    }
    else
    {
      *(reversedNodeList + i) = *(tailPtr - i);
    }
  }
}

// Helper function to check if a node destination is in the list
uint8_t nodeReachable(neighbor_list_S * neighborList, uint8_t nodeId)
{
  for (uint8_t idx = 0; idx < MAX_NUM_NEIGHBORS; ++idx)
  {
    if (neighborList->node_ids[idx] == nodeId)
    {
      return neighborList->edge_costs[idx];
    }
  }
  return 0; // Node is not in the list or is not reachable
}

// Helper function to change radio power levels
void changeRadioPowerLevel(edge_costs_E * powerLevel)
{
  switch (*powerLevel)
  {
    case MIN_POWER:
      radio.setPALevel(RF24_PA_MIN);
      break;
    case LOW_POWER:
      radio.setPALevel(RF24_PA_LOW);
      break;
    case HIGH_POWER:
      radio.setPALevel(RF24_PA_HIGH);
      break;
    case MAX_POWER:
      radio.setPALevel(RF24_PA_MAX);
      break;
    default: // Infinity power level or error (garbage)
      break;
  }
}


void fixList(uint8_t* origList)
{
  uint8_t dataIdx = 0;
  for (uint8_t idx = 0; idx < 9; ++idx)
  {
    if (origList[idx] != 0)
    {
      dataIdx = idx;
      break;
    }
  }
  if (dataIdx == 0)
  {
    return;
  }
  else
  {
    for (uint8_t idx = 0; idx < 9-dataIdx; ++idx)
    {
      origList[idx] = origList[dataIdx+idx];
    }
  }
}

// -------------- MAIN FUNCTIONS --------------
void setup()
{
#ifdef USE_RF24_RADIO
  radio.begin();

  // Set the PA Level low to prevent power supply related issues, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_MIN);
  // Set the data rate and ack channels
  radio.setChannel(ACK_CHANNEL);
  radio.enableDynamicPayloads();
  // Open a transmit and receive pipe
  radio.openWritingPipe(pipe);
  radio.openReadingPipe(1, pipe);



  // Start the radio listening for data
  radio.startListening();
#endif

  Serial.begin(115200); // This is a temp replacement for the radio interface
}

void loop()
{
  static states_E currentState = WAIT_FOR_TOKEN; // Initial state is wait for token
  static message_S currentMessage; // Set the current state to the starting node state
  static MsgPayload msgOutgoingPayloads; // Union of all possible message payloads for outgoing messages
  static MsgPayload msgIncomingPayloads; // Union of all possible message payloads for incoming messages
  static uint8_t numMsgRetries = 3; // How many times resend the message if no ack is received
  static uint32_t startListeningTimestamp = 0; // When this node began listening for responses
  static uint32_t listenMaxTime = 5000; // Time we should spend listening for responses
  static uint8_t lastMsgReversedNodePath[MAX_NODE_PATH] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t receivedMsgNodeId = 0; // The node id of the node that sent the message
  static message_S msgReceived; // Initialize an empty message put the received data in
  static message_S msgResponse; // Initialize an empty message
  static message_S msgAck; // Initialize an empty ack
  static uint8_t nList_idx = 0;
  static neighbor_list_S nList;
  bool computerMessage = false;
  static bool receivedNeighborQuery = false;
  uint8_t powerLevelToReachNode = 0;
  uint8_t rxBufferPtr = 0;
  uint8_t lastCharRead = '0';

  // Read in current message in the Serial buffer
  if (Serial.available())
  {
    uint32_t startTimeStamp = micros();
    while ((lastCharRead != '\n') && (((uint32_t)(micros() - startTimeStamp)) < 5000)) // Wait of end of message character or newline character
    {
      if (Serial.available())
      {
        lastCharRead = Serial.read();
        receiveBuffer[rxBufferPtr] = lastCharRead;
        ++rxBufferPtr;
      }
    }
  }
  if (rxBufferPtr != 0)
  {
    // Read in the serial buffa, read fixed incoming payload length
    //    Serial.readBytes((uint8_t*)&receiveBuffer, 15);

    //    sprintf(str, "%d", NODE_ID)
    if (receiveBuffer[2] != (NODE_ID + '0')) // If the message is not intended for this node
    {
      message_id_E messageType = (message_id_E) (receiveBuffer[0] - '0');
      // Forward the message
      if (messageType == DATA_QUERY)
      {
#ifdef SERIAL_DEBUG
        Serial.print(F("Forwarding DATA_QUERY message to node "));
        Serial.println(receiveBuffer[2]-'0');
#endif

        // Check if the node is reachable
        powerLevelToReachNode = nodeReachable(&nList, receiveBuffer[2] - '0');

        if (powerLevelToReachNode == INFINITY_POWER) // Node unreachable, send error message back
        {
          // Build and send the error message indicating the unreachable node
          msgOutgoingPayloads.errorMsgPayload.unreachable_node = receiveBuffer[2] - '0';

          // Forward the startup message, build the startup message
          buildMessage(&msgResponse, NODE_ID, ERROR_MSG, (uint8_t*) & (msgOutgoingPayloads.errorMsgPayload));
        }
        else // The node is reachable, forward the message
        {
          // Change power levels to reach it
          changeRadioPowerLevel((edge_costs_E*)&powerLevelToReachNode);

          // Translate the node path
          for (uint8_t idx = 0; idx < MAX_NODE_PATH; ++idx)
          {
            msgOutgoingPayloads.dataQueryPayload.node_path[idx] = receiveBuffer[idx + 4] - '0';
          }

          // Set the outgoing message payload equal to the incoming message payload
          msgOutgoingPayloads.dataQueryPayload.request = receiveBuffer[14] - '0';

          // Increment the target node index to the next node
          msgOutgoingPayloads.dataQueryPayload.target_node = 2; // MATLAB = 9, base node = 8

          // Forward the startup message, build the startup message
          buildMessage(&msgResponse, NODE_ID, DATA_QUERY, (uint8_t*) & (msgOutgoingPayloads.dataQueryPayload));

          // Send the message that is the size of the header + startup message payload
          sendMessage(&msgResponse, false);
        }
      }
      else if (messageType == STARTUP_MSG) // Forward the startup message
      {
#ifdef SERIAL_DEBUG
        Serial.print(F("Forwarding STARTUP_MSG message intended for node "));
        Serial.println(receiveBuffer[2]-'0');
#endif

        // Check if the node is reachable
        powerLevelToReachNode = nodeReachable(&nList, receiveBuffer[2] - '0');

        if (powerLevelToReachNode == INFINITY_POWER) // Node unreachable, send error message back
        {
          // Build and send the error message indicating the unreachable node
          msgOutgoingPayloads.errorMsgPayload.unreachable_node = receiveBuffer[2] - '0';

          // Forward the startup message, build the startup message
          buildMessage(&msgResponse, NODE_ID, ERROR_MSG, (uint8_t*) & (msgOutgoingPayloads.errorMsgPayload));
        }
        else // The node is reachable, forward the message
        {
          // Translate the node path
          for (uint8_t idx = 0; idx < MAX_NODE_PATH; ++idx)
          {
            msgOutgoingPayloads.startupMsgPayload.node_path[idx] = receiveBuffer[idx + 4] - '0';
          }

          // Increment the target node index to the next node
          msgOutgoingPayloads.startupMsgPayload.target_node = 2; // MATLAB = 9, base node = 8

          // Forward the startup message, build the startup message
          buildMessage(&msgResponse, NODE_ID, STARTUP_MSG, (uint8_t*) & (msgOutgoingPayloads.startupMsgPayload));

          // Change power levels to reach it
          changeRadioPowerLevel((edge_costs_E*)&powerLevelToReachNode);

          // Send the message that is the size of the header + startup message payload
          sendMessage(&msgResponse, false);
        }
      }
    }
    else // This message is intended for the basestation
    {
      // Check the message type
      if (receiveBuffer[0] == (STARTUP_MSG + '0')) // ALERT Check that this works
      {
        // Jump to startup mode
        currentState = STARTUP_MODE;
#ifdef SERIAL_DEBUG
        Serial.println(F("Received startup_msg, transitioning to STARTUP_MODE"));
#endif
        // Received startup message intended for this base node
        //        msgIncomingPayloads.startupMsgPayload.target_node = 1; // Should be second and last id in array
        //        msgIncomingPayloads.startupMsgPayload.node_path[0] = MATLAB_NODE_ID;
        //        msgIncomingPayloads.startupMsgPayload.node_path[1] = NODE_ID;
        //         TODO fill rest with 0s

        //        for (uint8_t idx = 0; idx < MAX_NODE_PATH; ++idx)
        //        {
        //          msgIncomingPayloads.startupMsgPayload.node_path[idx] = receiveBuffer[idx + 4]; // Node path will always start at slot 4
        //        }
        //
        //        buildMessage(&currentMessage, MATLAB_NODE_ID, STARTUP_MSG, (uint8_t*)(msgIncomingPayloads.startupMsgPayload));
        //        computerMessage = true; // Indicate that this message is from MATLAB
      }
    }
  }

  // This is the state machine for the node
  switch (currentState)
  {
    case WAIT_FOR_TOKEN:
      {
        // Listen for message on Main_Channel
        message_id_E messageType = TYPE_NONE;

        // Check the radio for an incoming message, if available, read from the buffer
        if (computerMessage || readMessage(&currentMessage))
        {
          debugMessage(&currentMessage); // DEBUG

          // Parse the header of the received messgae
          parseHeader(&currentMessage, &messageType, &receivedMsgNodeId);

          // If the message type is a neighbor query message
          if (messageType == NEIGHBOR_QUERY)
          {
#ifdef SERIAL_DEBUG
            Serial.println("Received NEIGHBOR_QUERY message");
#endif

            // Parse the payload into the incoming payloads union structure
            parsePayload(&currentMessage, &msgIncomingPayloads);

            // If this is the first neighbor query message heard
            if (!receivedNeighborQuery)
            {
              msgOutgoingPayloads.nRspPayload.querying_node_id = receivedMsgNodeId;
              msgOutgoingPayloads.nRspPayload.received_power = msgIncomingPayloads.nQueryPayload.broadcast_power;

              changeRadioPowerLevel((edge_costs_E*) &msgIncomingPayloads.nQueryPayload.broadcast_power);

              // Build and send message response
              buildMessage(&msgResponse, NODE_ID, NEIGHBOR_RSP, (uint8_t*) & (msgOutgoingPayloads.nRspPayload));
              receivedNeighborQuery = true;
            }

            // The last neighbor query has been heard, time to send the response
            if (msgIncomingPayloads.nQueryPayload.broadcast_power != MAX_POWER)
            {
              break;
            }

            // Reset the initial neighbor query indicator
            receivedNeighborQuery = false;

            // (Re)send NEIGHBOR_RESPONSE message randomly until ACK
            // If no ack is received after three retries
            if (!waitForAck(&msgResponse, NEIGHBOR_RSP_ACK, numMsgRetries, 2000))
            {
              // TODO Log the neighbor response failure
#ifdef SERIAL_DEBUG
              Serial.println("WARNING: No neighbor rsp ack received!");
#endif
            }
            else // Ack was received
            {
              // Read the message from the buffer
              if (!readMessage(&msgReceived)) // TODO Do we really want to be reading messages in multiple places
              {
#ifdef SERIAL_DEBUG
                Serial.println("ERROR: There was a problem reading in the received message!");
#endif
              }
              else // Check that the correct node acked the response
              {
                debugMessage(&msgReceived);
                if (getNodeIdFromHeader(msgReceived.header) != receivedMsgNodeId)
                {
#ifdef SERIAL_DEBUG
                  Serial.println("WARNING: Received neighbor response ack from incorrect node!");
#endif
                }

                // Parse the payload into the incoming payloads union structure
                parsePayload(&msgReceived, &msgIncomingPayloads);

                // Check that the ack was intended for this node
                if (msgIncomingPayloads.nRspPayloadAck.node_acknowledged != NODE_ID)
                {
#ifdef SERIAL_DEBUG
                  Serial.println("WARNING: This node's ID does not match the ack's intended node ID!");
#endif
                }
              }
            }
          }
          // If the message is a startup message
          else if (messageType == STARTUP_MSG)
          {
            // Parse the payload into the incoming payloads union structure
            parsePayload(&currentMessage, &msgIncomingPayloads);
#ifdef SERIAL_DEBUG
            Serial.print(F("STARTUP_MSG received,"));
#endif
            // Check if this node is the message's intended destination and that there is no next hop, indicated by the next node in the array being 0
            if ((msgIncomingPayloads.startupMsgPayload.node_path[msgIncomingPayloads.startupMsgPayload.target_node] == NODE_ID) &&
                ((msgIncomingPayloads.startupMsgPayload.target_node + 1 >= MAX_NODE_PATH) ||
                 (msgIncomingPayloads.startupMsgPayload.node_path[msgIncomingPayloads.startupMsgPayload.target_node + 1] == 0)))
            {
#ifdef SERIAL_DEBUG
              Serial.println(F(" transitioning to startup mode..."));
#endif

              // Store the reversed node path for when we need to send the startup response
              reverseNodeList((uint8_t*) & (msgIncomingPayloads.startupMsgPayload.node_path), (uint8_t*)&lastMsgReversedNodePath, MAX_NODE_PATH);
              fixList((uint8_t*) & (msgIncomingPayloads.startupMsgPayload.node_path));
              
              // This node is the end of the node path
              currentState = STARTUP_MODE;
            }
          }
        }
      }
      break;

    case STARTUP_MODE:
      {
        // Zero pad the neighbor list
        for (; nList_idx < MAX_NUM_NEIGHBORS; ++nList_idx)
        {
          nList.node_ids[nList_idx] = 0;
          nList.edge_costs[nList_idx] = 0;
        }
        nList_idx = 0;
        // MATLAB is always a neighbor
        nList.node_ids[nList_idx] = MATLAB_NODE_ID;
        nList.edge_costs[nList_idx] = MIN_POWER;
        ++nList_idx;

        // Broadcast query at all four power levels MIN, LOW, HIGH, MAX
        for (uint8_t powerLevel = 1; powerLevel < 5; ++powerLevel) // 5 is total number of edge costs
        {
          msgOutgoingPayloads.nQueryPayload.broadcast_power = powerLevel;

          // Start querying for neighbors, build neighbor query mesage
          buildMessage(&msgResponse, NODE_ID, NEIGHBOR_QUERY, (uint8_t*) & (msgOutgoingPayloads.nQueryPayload));
#ifdef SERIAL_DEBUG
          Serial.print(F("Broadcasting NEIGHBOR_QUERY at power level "));
          Serial.println(powerLevel);
#endif

          // Broadcast neighbor query message with size of header
          sendMessage(&msgResponse, false);
          delay(500);
        } // End of broadcasting NEIGHBOR_QUERY at different power levels

#ifdef SERIAL_DEBUG
        Serial.println(F("Listening for neighbor responses..."));
#endif

        // Listen for NEIGHBOR_RESPONSEs for a fixed time interval
        startListeningTimestamp = millis();
        while ((uint32_t)(millis() - startListeningTimestamp) < 4000) // Spends 5 seconds listening for a response
        {
#ifdef SERIAL_DEBUG
//          Serial.println(F("Radio silence"));
#endif
          // Wait for response from discovered neighbor, resend neighbor query if necessary
          if (radio.available())
          {
#ifdef SERIAL_DEBUG
            Serial.println(F("Message received!"));
#endif
            // Read the message from the buffer
            if (!readMessage(&msgReceived))
            {
#ifdef SERIAL_DEBUG
              Serial.println(F("ERROR: There was a problem reading the received message!"));
#endif
            }
            else // Message successfully read from the buffer
            {
              message_id_E messageType = TYPE_NONE;
              parseHeader(&msgReceived, &messageType, &receivedMsgNodeId);

              if (messageType != NEIGHBOR_RSP) // Wrong message received, throw it away and continue
              {
                #ifdef SERIAL_DEBUG
                Serial.println(F("DID NOT RECEIVE NEIGHBOR RESPONSE!"));
                debugMessage(&msgReceived);
                #endif
                continue;
              }

              // Parse the payload of the neighbor response message
              parsePayload(&msgReceived, &msgIncomingPayloads);

              // Check that the message contains the correct querying node id
              if (msgIncomingPayloads.nRspPayload.querying_node_id != NODE_ID)
              {
#ifdef SERIAL_DEBUG
                Serial.println(F("WARNING: Message contains incorrect querying_node_id"));
#endif
              }
              else // The message contents are correct
              {
                #ifdef SERIAL_DEBUG
                Serial.println(F("Neighbor response received adding edge to neighbor list"));
                #endif

                nList_idx = getNodeIdFromHeader(msgReceived.header);
                
                // Store the node id of the discovered neighbor in this node's neighbor list
                nList.node_ids[nList_idx] = getNodeIdFromHeader(msgReceived.header);

                // Store the power level of the discovered neighbor in this node's neighbor list
                nList.edge_costs[nList_idx] = msgIncomingPayloads.nRspPayload.received_power;

//                ++nList_idx; // Increment the list index

                // Set the payload neighbor response ack to the id of the node that we are acknowledging
                msgOutgoingPayloads.nRspPayloadAck.node_acknowledged = getNodeIdFromHeader(msgReceived.header);

                // Build and send NEIGHBOR_RSP_ACK when the NEIGHBOR_RSP received
                buildMessage(&msgAck, NODE_ID, NEIGHBOR_RSP_ACK, (uint8_t*) & (msgOutgoingPayloads.nRspPayloadAck));

                // Write the message ack with size of header + payload
                sendMessage(&msgAck, false);
              }
            }
          }
          // TODO Add print statement here to show that no neighbors responded
        } // End of listening loop

        // Reset the node list index for next time
//        nList_idx = 0;

        // NEIGHBORS QUERIED, TIME TO SEND NEIGHBOR LIST TO BASESTATION

        // Building payload: Set the node list index to the first neighbor
        msgOutgoingPayloads.startupRspPayload.target_node = 1;

        // Buidling payload: Save the reversed copy of the node_path that the original startup message contained
        memcpy(&(msgOutgoingPayloads.startupRspPayload.node_path), &lastMsgReversedNodePath, MAX_NODE_PATH);
        memcpy(&(msgOutgoingPayloads.startupRspPayload.node_ids), &(nList.node_ids), MAX_NODE_PATH);
        memcpy(&(msgOutgoingPayloads.startupRspPayload.edge_costs), &(nList.edge_costs), MAX_NODE_PATH);
//        msgOutgoingPayloads.startupRspPayload.neighbor_list = &nList; // Add the current neighbor list to the message

        // Build the startup response message
        buildMessage(&msgResponse, NODE_ID, STARTUP_RSP, (uint8_t*) & (msgOutgoingPayloads.startupRspPayload));

        // Send the startup response message with the size of the header and startup response payload
        sendMessage(&msgResponse, true); // Always send to basestation!

        // ALL done send STATUP RESPONSE (complete) message, "pass the token back to the base"
        currentState = NORMAL_MODE;
#ifdef SERIAL_DEBUG
        Serial.println(F("Transitioning to NORMAL mode"));
#endif
      }
      break;

    case NORMAL_MODE:
      {
        // Listen for message on Main_Channel
        if (computerMessage || readMessage(&currentMessage))
        {
          message_id_E messageType = TYPE_NONE;
          // Parse the contents of the received messgae
          parseHeader(&currentMessage, &messageType, &receivedMsgNodeId);

          // If the message type is a neighbor query message
          if (messageType == NEIGHBOR_QUERY)
          {
#ifdef SERIAL_DEBUG
            Serial.println(F("Received NEIGHBOR_QUERY message"));
#endif
            // Parse the payload into the incoming payloads union structure
            parsePayload(&currentMessage, &msgIncomingPayloads);

            // If this is the first neighbor query message heard
            if (!receivedNeighborQuery)
            {
              msgOutgoingPayloads.nRspPayload.querying_node_id = receivedMsgNodeId;
              msgOutgoingPayloads.nRspPayload.received_power = msgIncomingPayloads.nQueryPayload.broadcast_power;

              changeRadioPowerLevel((edge_costs_E*) &msgIncomingPayloads.nQueryPayload.broadcast_power);

              // Build and send message response
              buildMessage(&msgResponse, NODE_ID, NEIGHBOR_RSP, (uint8_t*) & (msgOutgoingPayloads.nRspPayload));
              receivedNeighborQuery = true;
            }

            // The last neighbor query has been heard, time to send the response
            if (msgIncomingPayloads.nQueryPayload.broadcast_power != MAX_POWER)
            {
              break;
            }

            // Reset the initial neighbor query indicator
            receivedNeighborQuery = false;
            uint8_t retry = 0;
            while (retry < 3)
            {
            // (Re)send NEIGHBOR_RESPONSE message randomly until ACK
            // If no ack is received after three retries
            if (!waitForAck(&msgResponse, NEIGHBOR_RSP_ACK, numMsgRetries, 2000))
            {
#ifdef SERIAL_DEBUG
              Serial.println("WARNING: No neighbor rsp ack received!");
#endif
            }
            else
            {
              // Read the message from the buffer
              if (!readMessage(&msgReceived)) // TODO Do we really want to be reading messages in multiple places
              {
#ifdef SERIAL_DEBUG
                Serial.println("ERROR: There was a problem reading in the received message!");
#endif
              }
              // Check that the correct node acked the response
              else
              {
                debugMessage(&msgReceived);
                if (getNodeIdFromHeader(msgReceived.header) != receivedMsgNodeId)
                {
#ifdef SERIAL_DEBUG
                  ++retry;
                  Serial.println("WARNING: Received neighbor response ack from incorrect node!");
#endif
                }
                else
                {
                  retry = 4;
                }

                // Parse the payload into the incoming payloads union structure
                parsePayload(&msgReceived, &msgIncomingPayloads);

                // Check that the ack was intended for this node
                if (msgIncomingPayloads.nRspPayloadAck.node_acknowledged != NODE_ID)
                {
#ifdef SERIAL_DEBUG
                  Serial.println("WARNING: This node's ID does not match the ack's intended node ID!");
#endif
                }
              }
            }
            } // END RETRY LOOP
          }
          else if (messageType == DATA_QUERY)
          {
            // Parse the payload into the incoming payloads union structure
            parsePayload(&currentMessage, &msgIncomingPayloads);

#ifdef SERIAL_DEBUG
            Serial.print("DATA_QUERY received,");
#endif
            // Check if this node is the message's intended destination and that there is no next hop, indicated by the next node in the array being 0
            if ((msgIncomingPayloads.dataQueryPayload.node_path[msgIncomingPayloads.dataQueryPayload.target_node] == NODE_ID) &&
                ((msgIncomingPayloads.dataQueryPayload.target_node + 1 >= MAX_NODE_PATH) ||
                 (msgIncomingPayloads.dataQueryPayload.node_path[msgIncomingPayloads.dataQueryPayload.target_node + 1] == 0)))
            {
#ifdef SERIAL_DEBUG
              Serial.println(" processing request...");
#endif

              // Store the reversed node path for when we need to send the startup response
              reverseNodeList((uint8_t*) & (msgIncomingPayloads.dataQueryPayload.node_path), (uint8_t*)&lastMsgReversedNodePath, MAX_NODE_PATH);
              fixList((uint8_t*) & (msgIncomingPayloads.dataQueryPayload.node_path));
              
              // TODO Process the request
              if (msgIncomingPayloads.dataQueryPayload.request == 1)
              {
#ifdef SERIAL_DEBUG
                Serial.println("Processing generic request 1!");
#endif
              }
              // TODO Send data back
            }
            else // This message should be forwarded
            {
#ifdef SERIAL_DEBUG
              Serial.print(" forwarding message to node ");
              Serial.println(msgIncomingPayloads.dataQueryPayload.node_path[msgIncomingPayloads.dataQueryPayload.target_node + 1], DEC);
#endif
              // Check that the next hop node has a valid id
              if (((msgIncomingPayloads.dataQueryPayload.target_node + 1) >= MAX_NODE_PATH) ||
                  (msgIncomingPayloads.dataQueryPayload.node_path[msgIncomingPayloads.dataQueryPayload.target_node + 1] == 0))
              {
#ifdef SERIAL_DEBUG
                Serial.println("ERROR: This node is not the destination, but end of node path reached!");
#endif
              }

              // Check if the node is reachable
              powerLevelToReachNode = nodeReachable(&nList, msgIncomingPayloads.dataQueryPayload.node_path[msgIncomingPayloads.dataQueryPayload.target_node + 1]);

              if (powerLevelToReachNode == INFINITY_POWER) // Node unreachable, send error message back
              {
                // Build and send the error message indicating the unreachable node
                msgOutgoingPayloads.errorMsgPayload.unreachable_node = msgIncomingPayloads.dataQueryPayload.node_path[msgIncomingPayloads.dataQueryPayload.target_node + 1];

                // Forward the startup message, build the startup message
                buildMessage(&msgResponse, NODE_ID, ERROR_MSG, (uint8_t*) & (msgOutgoingPayloads.errorMsgPayload));
              }
              else // The node is reachable, forward the message
              {
                // Set the outgoing message payload equal to the incoming message payload
                msgOutgoingPayloads.dataQueryPayload = msgIncomingPayloads.dataQueryPayload;

                // Increment the target node index to the next node
                ++(msgOutgoingPayloads.dataQueryPayload.target_node);

                // Forward the startup message, build the startup message
                buildMessage(&msgResponse, NODE_ID, DATA_QUERY, (uint8_t*) & (msgOutgoingPayloads.dataQueryPayload));

                changeRadioPowerLevel((edge_costs_E*)&powerLevelToReachNode);

                // Send the message that is the size of the header + startup message payload
                sendMessage(&msgResponse, false);
              }
            }
          }
          // Forward all data responses, startup, and startup response messages
          else if (messageType == DATA_RSP)
          {
            // Parse the payload into the incoming payloads union structure
            parsePayload(&currentMessage, &msgIncomingPayloads);
#ifdef SERIAL_DEBUG
            Serial.print("GForwarding message to node ");
            Serial.println(msgIncomingPayloads.dataRspPayload.node_path[msgIncomingPayloads.dataRspPayload.target_node + 1], DEC);
#endif
            // Check that the next hop node has a valid id
            if (((msgIncomingPayloads.dataRspPayload.target_node + 1) >= MAX_NODE_PATH) ||
                (msgIncomingPayloads.dataRspPayload.node_path[msgIncomingPayloads.dataRspPayload.target_node + 1] == 0))
            {
#ifdef SERIAL_DEBUG
              Serial.println("ERROR: This node is not the destination, but end of node path reached!");
#endif
            }
            // Set the outgoing message payload equal to the incoming message payload
            msgOutgoingPayloads.dataRspPayload = msgIncomingPayloads.dataRspPayload;

            // Increment the target node index to the next node
            ++(msgOutgoingPayloads.dataRspPayload.target_node);

            // Forward the startup message, build the startup message
            buildMessage(&msgResponse, NODE_ID, DATA_RSP, (uint8_t*) & (msgOutgoingPayloads.dataRspPayload));

            // Send the message that is the size of the header + startup message payload
            sendMessage(&msgResponse, true); // Always send data responses to MATLAB
          }
          else if (messageType == STARTUP_MSG)
          {
            // Parse the payload into the incoming payloads union structure
            parsePayload(&currentMessage, &msgIncomingPayloads);
#ifdef SERIAL_DEBUG
            Serial.print("STARTUP_MSG received,");
#endif
            // Check if this node is the message's intended destination and that there is no next hop, indicated by the next node in the array being 0
            if ((msgIncomingPayloads.startupMsgPayload.node_path[msgIncomingPayloads.startupMsgPayload.target_node] == NODE_ID) &&
                ((msgIncomingPayloads.startupMsgPayload.target_node + 1 >= MAX_NODE_PATH) ||
                 (msgIncomingPayloads.startupMsgPayload.node_path[msgIncomingPayloads.startupMsgPayload.target_node + 1] == 0)))
            {
#ifdef SERIAL_DEBUG
              Serial.println(" transitioning to startup mode...");
#endif
              // Store the reversed node path for when we need to send the startup response
              reverseNodeList((uint8_t*)&(msgIncomingPayloads.startupMsgPayload.node_path), (uint8_t*)&lastMsgReversedNodePath, MAX_NODE_PATH);
              fixList((uint8_t*)&(msgIncomingPayloads.startupMsgPayload.node_path));
              
              // This node is the end of the node path
              currentState = STARTUP_MODE;
              break;
            }

#ifdef SERIAL_DEBUG
            debugMessage(&currentMessage);
            Serial.print("LForwarding message to node ");
            Serial.println(msgIncomingPayloads.startupMsgPayload.node_path[msgIncomingPayloads.startupMsgPayload.target_node + 1], DEC);
#endif
            // Check that the next hop node has a valid id
            if (((msgIncomingPayloads.startupMsgPayload.target_node + 1) >= MAX_NODE_PATH) ||
                (msgIncomingPayloads.startupMsgPayload.node_path[msgIncomingPayloads.startupMsgPayload.target_node + 1] == 0))
            {
#ifdef SERIAL_DEBUG
              Serial.println("ERROR: This node is not the destination, but end of node path reached!");
#endif
            }

            // Check if the node is reachable
            powerLevelToReachNode = nodeReachable(&nList, msgIncomingPayloads.startupMsgPayload.node_path[msgIncomingPayloads.startupMsgPayload.target_node+1]);

            if (powerLevelToReachNode == INFINITY_POWER) // Node unreachable, send error message back
            {
              // Build and send the error message indicating the unreachable node
              msgOutgoingPayloads.errorMsgPayload.unreachable_node = msgIncomingPayloads.startupMsgPayload.node_path[msgIncomingPayloads.startupMsgPayload.target_node + 1];

              // Forward the startup message, build the startup message
              buildMessage(&msgResponse, NODE_ID, ERROR_MSG, (uint8_t*) & (msgOutgoingPayloads.errorMsgPayload));
            }
            else // The node is reachable, forward the message
            {
              // Set the outgoing message payload equal to the incoming message payload
              memcpy((uint8_t*)&(msgOutgoingPayloads.startupMsgPayload), (uint8_t*)&(msgIncomingPayloads.startupMsgPayload), sizeof(startup_msg_payload_S));

              // Increment the target node index to the next node
              ++(msgOutgoingPayloads.startupMsgPayload.target_node);

              // Forward the startup message, build the startup message
              buildMessage(&msgResponse, NODE_ID, STARTUP_MSG, (uint8_t*) & (msgOutgoingPayloads.startupMsgPayload));

              // Send the message that is the size of the header + startup message payload
              sendMessage(&msgResponse, false); // Do not send startup messages to matlab
            }
          }
          else if (messageType == STARTUP_RSP)
          {
            // Parse the payload into the incoming payloads union structure
            parsePayload(&currentMessage, &msgIncomingPayloads);
            
#ifdef SERIAL_DEBUG
            Serial.print("HForwarding message to node ");
            Serial.println(msgIncomingPayloads.startupRspPayload.node_path[msgIncomingPayloads.startupRspPayload.target_node + 1], DEC);
#endif
            // Check that the next hop node has a valid id
            if (((msgIncomingPayloads.startupRspPayload.target_node + 1) >= MAX_NODE_PATH) ||
                (msgIncomingPayloads.startupRspPayload.node_path[msgIncomingPayloads.startupRspPayload.target_node + 1] == 0))
            {
#ifdef SERIAL_DEBUG
              Serial.println("ERROR: This node is not the destination, but end of node path reached!");
#endif
            }
            // Set the outgoing message payload equal to the incoming message payload
            memcpy((uint8_t*)&(msgOutgoingPayloads.startupRspPayload), (uint8_t*)&(msgIncomingPayloads.startupRspPayload), sizeof(startup_rsp_payload_S));
            
            // Increment the target node index to the next node
            ++(msgOutgoingPayloads.startupRspPayload.target_node);

            // Forward the startup message, build the startup message
            buildMessage(&msgResponse, NODE_ID, STARTUP_RSP, (uint8_t*) & (msgOutgoingPayloads.startupRspPayload));
            debugMessage(&msgResponse);
            // Send the message that is the size of the header + startup message payload
            sendMessage(&msgResponse, true); // Always forward startup responses to MATLAB
          }
        }
      }
      break;

    default:
      {
#ifdef SERIAL_DEBUG
        Serial.println("ERROR: Reached unknown state!");
#endif
      }
      break;
  } // End of state machine
} // End of loop
