/** DSSN Final Project
 * Node code
 */

// -------------- DEFINES --------------
// Unique node identifier
#define NODE_ID 1U

// Maximum number of nodes
#define MAX_NODE_NUM 50U

// Declare channel ID
// What will this look like?

// Declare message properties
#define MSG_NODE_ID_OFFSET 4
#define MSG_NODE_ID_MASK 0xF0
#define MSG_TYPE_MASK 0x0F

// -------------- STRUCTS & ENUMS --------------
// Message packet structure
typedef struct __attribute__((__packed__))
{
    uint8_t header = 0; // First (Most Significant) 4 bits = Node ID, Second 4 bits = Message type
    uint8_t payload = 0;
} message_S;

// Enumeration of possible node states
typedef enum
{
  WAIT_FOR_INIT,
  INIT,
  NORMAL
} states_E;

// Message IDs
typedef enum
{
  NEIGHBOR_QUERY, // "Hello!"
  NEIGHBOR_QUERY_ACK, // "Hello back at cha!" or ack
  STARTUP_MSG, // "Wake up!"
  NORMAL_MSG
} message_id_E;

// -------------- GLOBAL VARIABLES --------------
// Set the current state to the starting node state
uint8_t currentState = WAIT_FOR_INIT;
message_S* currentMessage = 0;

// Initialize the node id queue
uint8_t node_id_queue[MAX_NODE_NUM];
uint8_t queue_index = 0;

// Initialize an empty list of neighbors
uint8_t node_neighbors[] = {};


// -------------- FUNCTIONS --------------
void buildMessage(message_S* msg, uint8_t node_id, uint8_t message_type, uint8_t payload)
{
  // Build the header with the node id and message type
  msg->header = ((node_id << MSG_NODE_ID_OFFSET) & MSG_NODE_ID_MASK) | (message_type & MSG_TYPE_MASK);

  // Build the message payload, more to come here...?
  msg->payload = payload;
}

void setup()
{
  // 1.3kB in the Serial object alone! 630 bytes all else
  Serial.begin(9600); // This is a temp replacement for the radio interface 
}

void loop() // 700 Bytes in main loop
{
  
  message_S msgResponse; // Initialize an empty message
  
  // TODO Read bytes from the radio over SPI
  if (Serial.available())
  {
    // Cast the byte stream as the message struct
    currentMessage = (message_S*) Serial.read(); // TODO This will be replaced with the radio interface
  }
  
  // How long to wait listening for neighbors?
  // Do we wait for sender to send clear to awaken signal?
  // Then if no neighbors, reply no neighbors found, awaken a different node

  // Check what type of message was received
  switch (currentMessage->header & MSG_TYPE_MASK)
  {
    case NEIGHBOR_QUERY: // Wait for initialization signal
      // Respond to neighbor query message with: 1) Node ID 2) Listening power
      
      
      static uint8_t payload = 0; // TODO put the listening power here?
      
      // Build and send message response
      buildMessage(&msgResponse, NODE_ID, NEIGHBOR_QUERY_ACK, payload);
      break;
      
    case STARTUP_MSG: // Begin initialization routine
      // Start the random timer 0-1 seconds
      static uint8_t waitTime = random(0,1000);
      delay(waitTime);
      // Listen to the medium -> TODO Check if the channel is idle
      // ASSUMING channel is idle else restart timer
      
       // Build and send the neighbor query message
      buildMessage(&msgResponse, NODE_ID, NEIGHBOR_QUERY, 0);

      // TODO Broadcast the message

      // TODO enter a wait for response state maybe while loop with timeout
      break;
      
    case NORMAL_MSG:
      break;
  }
  
}
