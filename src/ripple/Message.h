#pragma once

#include "Thread.h"
#include "graph/map.h"

// Message type
enum MessageType {
  MESSAGE_COLLISION, // Sent by other threads to the source thread during phase
                     // 1 when a collision happens
  MESSAGE_PHASE_2,   // Sent by the source thread to signal other threads that
                     // phase 2 has started
  MESSAGE_STOP, // Sent by the source thread to signal an other thread that it
                // has no work to do in phase 2
  MESSAGE_DONE, // Sent by other threads to the source thread to signal that
                // they finished their phase 2 work
};

// Struct representing a message that is sent between ripple threads
// the valid fields of the union depend on the message type
struct Message {
  MessageType type;

  union {
    struct {
      Node source;
      Node target;
    } phase_info; // type = MESSAGE_PHASE_2 for worker threads

    Node final_node; // type = MESSAGE_PHASE_2 for goal thread

    struct {
      Node collision_node;   // Node that is already owned by the target thread
      Node collision_parent; // Node that the source thread came from when he
                             // found the collision
      ThreadId collision_source; // Thread that found the collision
      ThreadId collision_target; // Thread that already owned the node
    } collision_info;            // type = MESSAGE_COLLISION
  };
};
