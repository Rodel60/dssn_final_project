/** DSSN Final Project
 * Node code
 */

// -------------- DEFINES --------------
// Unique node identifier
// NOTE: Start incrementing the node id from 1, so 0 represents no node
#define NODE_ID 1U

// Maximum number of nodes TODO Is this even used anymore?
#define MAX_NODE_NUM 50U

// Maximum number of nodes in a message path
#define MAX_NODE_PATH 9U

// Maximum possible number of neighbor nodes
#define MAX_NUM_NEIGHBORS 9U

// Empty message payload
#define EMPTY_MSG_PAYLOAD 0U

// Declare channel ID
// What will this look like?

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
{ // TODO Possibly set them all to 0
  uint8_t node_ids[MAX_NUM_NEIGHBORS]; // These two arrays are alligned by id and cost
  uint8_t edge_costs[MAX_NUM_NEIGHBORS]; // Edge costs represent power required to reach the node
} neighbor_list_S;

// Message Payload Structs
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
  neighbor_list_S neighbor_list;
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
  uint8_t request; // TODO Create enum of request numbers
} data_rsp_payload_S;

// DATA_RSP_ACK Payload
typedef struct __attribute__((__packed__))
{
  uint8_t target_node;
} data_rsp_ack_payload_S;

// Enumeration of possible node states
typedef enum
{
  WAIT_FOR_TOKEN,
  STARTUP_MODE,
  NORMAL_MODE
} states_E;

union MsgPayload // Declare a simple union type  
{  
    neighbor_rsp_payload_S* nRspPayload;
    neighbor_rsp_ack_payload_S* nRspPayloadAck;
    startup_msg_payload_S* startupMsgPayload;
    startup_rsp_payload_S* startupRspPayload;
    data_query_payload_S* dataQueryPayload;
    data_rsp_payload_S* dataRspPayload;
    data_rsp_ack_payload_S* dataRspAckPayload;
};

// Message IDs
typedef enum
{
  NEIGHBOR_QUERY = 0, // "Hello!"
  NEIGHBOR_RSP = 1, // Hello I heard you
  NEIGHBOR_RSP_ACK = 2, // I got your response, no need to resend
  STARTUP_MSG = 3, // Begin startup
  STARTUP_RSP = 4, // Startup complete
  DATA_QUERY = 5, // Tell me some info
  DATA_RSP = 6, // Info received
  DATA_RSP_ACK = 7, // Ack that info was received, no need to resend
  TYPE_NONE = 8
} message_id_E;



// -------------- GLOBAL VARIABLES --------------
// Initialize the node id queue
uint8_t nList_idx = 0;
neighbor_list_S nList; // TODO move this to the main loop
uint8_t payloadDataBuffer[64]; // Max serial buffer data size

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

// Function to check the radio for messages
bool checkForMessageType(message_id_E typeToCheck)
{
  message_id_E messageType = TYPE_NONE;
  uint8_t currMsgHeader;
  
  // TODO Read bytes from the radio over SPI?
  if (Serial.available())
  {
    // Cast the byte stream as the message struct
    currMsgHeader = Serial.peek(); // Peek the header
    if (getMessageIdFromHeader(currMsgHeader) == typeToCheck)
    {
      Serial.println("Correct ACK received!");
      return true; // Read in a message of the proper type
    }
  }
  // All other cases return false
  return false;
}

// Helper function to calculate the number of bytes to expect in a message payload
uint8_t getPayloadSize(message_id_E msgType)
{
  uint8_t numPayloadBytes = 0;
  switch (msgType)
  {
    case NEIGHBOR_QUERY:
      numPayloadBytes = EMPTY_MSG_PAYLOAD;
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

    default:
      Serial.println("[getPayloadSize] ERROR: Unknown message type!");
  }
  return numPayloadBytes;
}

// Function to check the radio for ack
bool readMessage(message_S* currMessage) // TODO Read bytes from the radio over SPI
{
  uint8_t payloadSize = 0;
  uint8_t numBytesInBuffer = Serial.available();

  // If there are bytes in the buffer
  if (numBytesInBuffer != 0)
  {
    // Read in the header
    Serial.readBytes(&(currMessage->header), sizeof(uint8_t));

    // Calculate the size of the expected payload
    payloadSize = getPayloadSize(getMessageIdFromHeader(currMessage->header));
    
//    Serial.print("PayloadSize = ");
//    Serial.println(payloadSize,DEC);
    
    // Read in the payload of the message
    Serial.readBytes((uint8_t*)&payloadDataBuffer, payloadSize);
    
    // Set the payload to the global payload buffer
    currMessage->payload = (uint8_t*)&payloadDataBuffer;
    return true;
  }
  else
  {
    currMessage = NULL;
    return false;
  }
}

// Debug function to display message contents
static void debugMessage(message_S* msgToDebug)
{
  MsgPayload debugMsgPayloads;
  message_id_E debugMsgType;
  uint8_t debugMsgNodeId = 0;
  
  parseHeader(msgToDebug, &debugMsgType, &debugMsgNodeId);
  Serial.print("Sent from Node ");
  Serial.println(debugMsgNodeId,DEC);
  
  parsePayload(msgToDebug, &debugMsgPayloads);
  
  Serial.print("Message type: ");
  switch (debugMsgType)
  {
    case NEIGHBOR_QUERY:
      Serial.println("NEIGHBOR_QUERY");
      break;

    case NEIGHBOR_RSP:
      Serial.println("NEIGHBOR_RSP");
      Serial.print("querying_node_id = ");
      Serial.println(debugMsgPayloads.nRspPayload->querying_node_id,DEC);
      Serial.print("received_power = ");
      Serial.println(debugMsgPayloads.nRspPayload->received_power,DEC);
      break;
      
    case NEIGHBOR_RSP_ACK:
      Serial.println("NEIGHBOR_RSP_ACK");
      Serial.print("node_acknowledged = ");
      Serial.println(debugMsgPayloads.nRspPayloadAck->node_acknowledged,DEC);
      break;
      
    case STARTUP_MSG:
      Serial.println("STARTUP_MSG");
      // TODO Add func as needed debugMsgPayloads.startupMsgPayload
      break;
      
    case STARTUP_RSP:
      Serial.println("STARTUP_RSP");
      // TODO Add func as needed debugMsgPayloads.startupRspPayload
      break;
      
    case DATA_QUERY:
      Serial.println("DATA_QUERY");
      // TODO Add func as needed debugMsgPayloads.dataQueryPayload
      break;
      
    case DATA_RSP:
      Serial.println("DATA_RSP");
      // TODO Add func as needed debugMsgPayloads.dataRspPayload
      break;
      
    case DATA_RSP_ACK:
      Serial.println("DATA_RSP_ACK");
      // TODO Add func as needed debugMsgPayloads.dataRspAckPayload
      break;
      
    case TYPE_NONE:
      Serial.println("TYPE_NONE");
      break;
      
    default:
      Serial.println("Unknown");
      break;
  } 
}

// Function to parse a received message header TODO rename parse header
void parseHeader(message_S* receivedMsg, message_id_E* messageType, uint8_t* txNodeId)
{
  // Mask out the message type
  *messageType = getMessageIdFromHeader(receivedMsg->header);
  *txNodeId = getNodeIdFromHeader(receivedMsg->header);
}

// Return true when the specified ack has been received, otherwise rebroadcast the message
bool waitForAck(message_S* messageToSend, message_id_E ackToReceive, uint8_t numRetries, uint32_t timeBetweenRetries)
{
  bool ackReceived = false;
  uint8_t numRetriesSent = 0;
  uint8_t randTime = random(256); // Wait a random amount of time 0-.25sec
  uint32_t startTimeStamp = 0; // Timer to indicate when to retry the message
  delay(randTime);
  Serial.println("Broadcasting intended message...");

  // Transmit the message
  sendMessage(messageToSend); // TODO Replace with radio interface

  startTimeStamp = millis(); // Start the timer
  
  // Check the radio for an incoming ack
  while(!checkForMessageType(ackToReceive)) // TODO what if there is garbage in the buffer?, need to flush it
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
        Serial.println("[No Ack] Broadcasting intended message again...");
        ++numRetriesSent; // Record the retry
        sendMessage(messageToSend); // Send the message again
        startTimeStamp = millis(); // Restart the timer
      }
    }
  }

  // If the expected ack is received
  return true;
}

// Wrapper function that handles message sizing when sending a message
void sendMessage(message_S* msgToSend)
{
  // Calculate size of the respective payload in bytes  
  uint8_t numBytesInPayload = getPayloadSize(getMessageIdFromHeader(msgToSend->header));
  
  // Send the message with the appropriate number of bytes
  Serial.write((uint8_t*)msgToSend, (sizeof(uint8_t)+numBytesInPayload));
}

// Parse the payload of the received message into the intended message struct
void parsePayload(message_S* receivedMsg, MsgPayload* receivedMsgPayloads)
{ // We're transforming the byte stream into a known data structure that we can easily access
  // Check the message type and cast the payload as appropriate payload struct
  switch (getMessageIdFromHeader(receivedMsg->header))
  {
    case NEIGHBOR_QUERY:
      // Do nothing because this message has no payload
      break;

    case NEIGHBOR_RSP:
      receivedMsgPayloads->nRspPayload = (neighbor_rsp_payload_S*)(receivedMsg->payload);
      break;

    case NEIGHBOR_RSP_ACK:
      receivedMsgPayloads->nRspPayloadAck = (neighbor_rsp_ack_payload_S*)(receivedMsg->payload);
      break;

    case STARTUP_MSG:
      receivedMsgPayloads->startupMsgPayload = (startup_msg_payload_S*)(receivedMsg->payload);
      break;

    case STARTUP_RSP:
      receivedMsgPayloads->startupRspPayload = (startup_rsp_payload_S*)(receivedMsg->payload);
      break;

    case DATA_QUERY:
      receivedMsgPayloads->dataQueryPayload = (data_query_payload_S*)(receivedMsg->payload);
      break;

    case DATA_RSP:
      receivedMsgPayloads->dataRspPayload = (data_rsp_payload_S*)(receivedMsg->payload);
      break;

    case DATA_RSP_ACK:
      receivedMsgPayloads->dataRspAckPayload = (data_rsp_ack_payload_S*)(receivedMsg->payload);
      break;

    default:
      // Error unrecognized message id
      Serial.println("ERROR: Unknown message ID!");
      break;
  }
}

// Helper function to reverse the node list
void reverseNodeList(uint8_t* origNodeList, uint8_t* reversedNodeList, uint8_t listSize)
{
  // Grab the end of the list
  uint8_t* tailPtr = (origNodeList + listSize - 1);

  // Write the end of the list into the front of the list
  for (uint8_t i=0; i<listSize; ++i)
  {
    // Ensure that zero padding is only at the end
    if (*(tailPtr-i) == 0)
    {
      *(reversedNodeList + (listSize-i-1)) = 0;
    }
    else
    {
      *(reversedNodeList + i) = *(tailPtr - i);
    }
  }
}

// -------------- MAIN FUNCTIONS --------------
void setup()
{
  Serial.begin(9600); // This is a temp replacement for the radio interface 
}

void loop()
{
  static bool messageReceived = false; // Boolean indicating if a message was received on the radio
  static states_E currentState = WAIT_FOR_TOKEN; // Initial state is wait for token
  static message_S currentMessage; // Set the current state to the starting node state
  static MsgPayload msgOutgoingPayloads; // Union of all possible message payloads for outgoing messages
  static MsgPayload msgIncomingPayloads; // Union of all possible message payloads for incoming messages
  static uint8_t numMsgRetries = 3; // How many times resend the message if no ack is received
  static uint32_t startListeningTimestamp = 0; // When this node began listening for responses
  static uint32_t listenMaxTime = 5000; // Time we should spend listening for responses
  static uint8_t lastMsgReversedNodePath[MAX_NODE_PATH] = {0,0,0,0,0,0,0,0,0};
  message_id_E receivedMsgType = TYPE_NONE;
  uint8_t receivedMsgNodeId = 0; // The node id of the node that sent the message
  message_S msgReceived; // Initialize an empty message put the received data in
  message_S msgResponse; // Initialize an empty message
  message_S msgAck; // Initialize an empty ack
  // This is the state machine for the node
  switch (currentState)
  {
    case WAIT_FOR_TOKEN:
    {
      // Listen for message on Main_Channel
      message_id_E messageType = TYPE_NONE;
      
      // Check the radio for an incoming message, if available, read from the buffer
      if (readMessage(&currentMessage))
      { 
        debugMessage(&currentMessage);
        // Parse the header of the received messgae
        parseHeader(&currentMessage, &messageType, &receivedMsgNodeId);

        // If the message type is a neighbor query message
        if (messageType == NEIGHBOR_QUERY)
        {
          Serial.println("Received NEIGHBOR_QUERY message");
          msgOutgoingPayloads.nRspPayload->querying_node_id = receivedMsgNodeId;
          msgOutgoingPayloads.nRspPayload->received_power = 255; // TODO Add in received power to payload
          
          // Build and send message response
          buildMessage(&msgResponse, NODE_ID, NEIGHBOR_RSP, (uint8_t*)(msgOutgoingPayloads.nRspPayload));

          // (Re)send NEIGHBOR_RESPONSE message randomly until ACK
          // If no ack is received after three retries
          if (!waitForAck(&msgResponse, NEIGHBOR_RSP_ACK, numMsgRetries, 2000))
          {
            // TODO Log the neighbor response failure
            Serial.println("WARNING: No neighbor rsp ack received!");
          }
          else
          {
            // Read the message from the buffer
            if (!readMessage(&msgReceived))
            {
              Serial.println("ERROR: There was a problem reading in the received message!");
              // ERROR there was a problem reading in the received message
            }
            // Check that the correct node acked the response
            else 
            {
              debugMessage(&msgReceived);
              if (getNodeIdFromHeader(msgReceived.header) != receivedMsgNodeId)
              {
                Serial.println("WARNING: Received neighbor response ack from incorrect node!");
              }

              // Parse the payload into the incoming payloads union structure
              parsePayload(&msgReceived, &msgIncomingPayloads);

              // Check that the ack was intended for this node
              if (msgIncomingPayloads.nRspPayloadAck->node_acknowledged != NODE_ID)
              {
                Serial.println("WARNING: This node's ID does not match the ack's intended node ID!");
              }
            }
          }
        }
        // If the message is a startup message
//        else if (messageType == STARTUP_MSG)
//        {
//          // Parse the payload into the incoming payloads union structure
//          parsePayload(&msgReceived, &msgIncomingPayloads);
//          Serial.print("STARTUP_MSG received,");
//
//          // Check if this node is the message's intended final destination
//          if (msgIncomingPayloads.startupMsgPayload->node_path[msgIncomingPayloads.startupMsgPayload->target_node] == NODE_ID)
//          {
//            Serial.println(" transitioning to startup mode...");
//
//            // Store the reversed node path for when we need to send the startup response
//            reverseNodeList((uint8_t*)&(msgIncomingPayloads.startupMsgPayload->node_path), (uint8_t*)&lastMsgReversedNodePath, MAX_NODE_PATH);
//
//            // This node is the end of the node path
//            currentState = STARTUP_MODE;
//          }
//          else
//          {
//            Serial.println(" forwarding message to next node. ");
//            if ((msgIncomingPayloads.startupMsgPayload->target_node + 1) > MAX_NODE_PATH)
//            {
//              Serial.println("ERROR: This node is not the destination, but end of node path reached!");
//            }
//            // Set the outgoing message payload equal to the incoming message payload
//            msgOutgoingPayloads.startupMsgPayload = msgIncomingPayloads.startupMsgPayload;
//            
//            // Increment the target node index to the next node
//            ++(msgOutgoingPayloads.startupMsgPayload->target_node);
//
//            // Forward the startup message, build the startup message
//            buildMessage(&msgResponse, NODE_ID, STARTUP_MSG, (uint8_t*)(msgOutgoingPayloads.startupMsgPayload));
//
//            // Send the message that is the size of the header + startup message payload
//            Serial.write((uint8_t*) &msgResponse, (sizeof(uint8_t)+sizeof(startup_msg_payload_S)));
//          }
//        }
      }
    }
    break;
      
//    case STARTUP_MODE:
//    {// TODO Ignore all other messages
//      // Start querying for neighbors
//      // Build neighbor query mesage
//      buildMessage(&msgResponse, NODE_ID, NEIGHBOR_QUERY, EMPTY_MSG_PAYLOAD);
//      Serial.println("Broadcasting NEIGHBOR_QUERY");
//      
//      // Broadcast neighbor query message with size of header
//      Serial.write((uint8_t*)&msgResponse, sizeof(uint8_t));
//      startListeningTimestamp = micros();
//      
//      Serial.println("Listening for neighbor responses...");
//      // Listen for NEIGHBOR_RESPONSEs for a fixed time interval
//      while ((uint32_t)(micros() - startListeningTimestamp) < listenMaxTime)
//      { 
//        // Wait for response from discovered neighbor, resend neighbor query if necessary
//        if (waitForAck(&msgResponse, NEIGHBOR_RSP, numMsgRetries, 500)) // MSH - Do we want to retry the neighbor query or is it one and done
//        {
//          Serial.println("NEIGHBOR_RSP received!");
//          // Read the message from the buffer
//          if (!readMessage(&msgReceived))
//          {
//            Serial.println("ERROR: There was a problem reading the received message!");
//            // ERROR there was a problem reading in the received message
//          }
//          // Message successfully read from the buffer
//          else
//          {
//            // Parse the payload of the neighbor response message
//            parsePayload(&msgReceived, &msgIncomingPayloads);
//
//            // Check that the message contains the correct querying node id
//            if (msgIncomingPayloads.nRspPayload->querying_node_id != NODE_ID)
//            {
//              Serial.println("WARNING: Message contains incorrect querying_node_id");
//            }
//            else // The message contents are correct
//            {
//              // Store the node id of the discovered neighbor in this node's neighbor list
//              nList.node_ids[nList_idx] = getNodeIdFromHeader(msgReceived.header);
//            
//              // Store the power level of the discovered neighbor in this node's neighbor list
//              nList.node_ids[nList_idx] = msgIncomingPayloads.nRspPayload->received_power;
//            
//              // Set the payload neighbor response ack to the id of the node that we are acknowledging
//              msgOutgoingPayloads.nRspPayloadAck->node_acknowledged = getNodeIdFromHeader(msgReceived.header);
//
//              // Build and send NEIGHBOR_RSP_ACK when the NEIGHBOR_RSP received
//              buildMessage(&msgAck, NODE_ID, NEIGHBOR_RSP_ACK, (uint8_t*)(msgOutgoingPayloads.nRspPayloadAck));
//              
//              // Write the message ack with size of header + payload
//              Serial.write((uint8_t*)&msgAck, (sizeof(uint8_t)+sizeof(neighbor_rsp_ack_payload_S))); 
//            }
//          }
//        }
//        // TODO Add print statement here to show that no neighbors responded
//      } // End of listening loop
//
//      // Building payload: Set the node list index to the first neighbor
//      msgOutgoingPayloads.startupRspPayload->target_node = 1;
//
//      // Buidling payload: Save the reversed copy of the node_path that the original startup message contained
//      memcpy(&(msgOutgoingPayloads.startupRspPayload->node_path), &lastMsgReversedNodePath, MAX_NODE_PATH);
//
//      // Building payload: Send in this node's neighbor list
//      memcpy(&(msgOutgoingPayloads.startupRspPayload->neighbor_list), &nList, sizeof(neighbor_list_S));
//      
//      // Build the startup response message
//      buildMessage(&msgResponse, NODE_ID, STARTUP_RSP, (uint8_t*)(msgOutgoingPayloads.startupRspPayload));
//      
//      // Send the startup response message with the size of the header and startup response payload
//      Serial.write((uint8_t*)&msgAck, (sizeof(uint8_t)+sizeof(startup_rsp_payload_S)));
//      
//      // ALL done send STATUP RESPONSE (complete) message, "pass the token back to the base"
//      currentState = NORMAL_MODE;
//    }
//    break;
    
    case NORMAL_MODE:
    {
      Serial.println("TODO Write normal mode");
      // Listen for message on Main_Channel
      /*if (messageReceived)
      {
        // Parse the contents of the received messgae
        parseHeader(currentMessage, &receivedMsgType, &receivedMsgNodeId);
        
        if (receivedMsgType == NEIGHBOR_QUERY)
        {
          payload = receivedMsgNodeId; // TODO add received power in the payload
          
          // Build and send message response
          buildMessage(&msgResponse, NODE_ID, NEIGHBOR_RSP, payload);

          // (Re)send NEIGHBOR_RESPONSE message randomly until ACK
          // If no ack is received after three retries
          if (!waitForAck(&msgResponse, NEIGHBOR_RSP_ACK, numMsgRetries, 2000))
          {
            // TODO Log the neighbor response failure
          }
          else
          {
            // Read the message from the buffer
            if (!checkForMessage(&msgReceived))
            {
              // ERROR there was a problem reading in the received message
            }
          }
        }
        else if (receivedMsgType == DATA_QUERY)
        {
          // TODO Process the normal message
        }
      }*/
    }
    break;

    default:
    {
      Serial.println("ERROR: Reached unknown state!");
    }
    break;
  }
}

