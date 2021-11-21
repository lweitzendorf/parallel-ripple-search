#pragma once

#include <stdint.h>
#include <atomic>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <oneapi/tbb/concurrent_queue.h>


#include "map.h"
#include "fringe.h"

#define NUM_THREADS 4

using oneapi::tbb::concurrent_queue;

enum ThreadId: int8_t {
    THREAD_NONE   = -1,
    THREAD_SOURCE = 0,
    THREAD_GOAL   = NUM_THREADS - 1,
};

// Struct representing a collision between threads.
// The source thread is not stored because it's implicitly the thread that sent the message
struct Collision {
    ThreadId target;       // Thread that we collided with
    Node collision_node;   // Node at which the collision happened
};

// Message type
enum MessageType {
    MESSAGE_COLLISION, // Sent by other threads to the source thread during phase 1 when a collision happens
    MESSAGE_PHASE_2,   // Sent by the source thread to signal other threads that phase 2 has started
    MESSAGE_STOP,      // Sent by the source thread to signal an other thread that it has no work to do in phase 2    
    MESSAGE_DONE,      // Sent by other threads to the source thread to signal that they finished their phase 2 work
};

// Struct representing a message that is sent between ripple threads
// the valid fields of the union depend on the message type
struct Message {
    MessageType type;
    union {
        struct {
            Node source;
            Node target;
        }; // type = MESSAGE_PHASE_2

        struct {
            Collision collision;
            ThreadId collision_source;
        }; // type = MESSAGE_COLLISION
    };
};


// TODO: test aligning this struct to cache line size to avoid
// false sharing between threads that are exploring neighbouring nodes
struct RippleCacheNode {
    // Id of the thread that first discovered the Node and thus owns it
    std::atomic<ThreadId> thread;

    // Fringe search data
    FringeNode node;
};

// Class representing a thread of the ripple search algorithm
class RippleThread {
private:
    std::unique_ptr<std::thread> thread;

    // Read only data:
    // My id
    ThreadId id;
    // internal thread

    // Map
    Map& map;
    // Message queues for sending to other threads
    std::vector<concurrent_queue<Message>>& message_queues;

    // Condition variable and mutex for putting the thread to sleep
    // when it has to wait for other threads
    std::mutex wait_mutex;
    std::condition_variable wait_cv;

    // Current source ang goals for the thread.
    // Slave threads have 2 goals because they search in both directions
    // until they have a collision with one of the two.
    // goal_2 == INVALID_NODE for the Source and Goal threads
    Node source;
    Node goal;
    Node goal_2;
    
    // Fringe search info
    FringeList fringe_list;
    int (*heuristic)(RippleThread* self, Node n);
    int flimit;

    // Referernce to vector of node info for fringe search, shared between all threads
    std::vector<RippleCacheNode>& cache;

    // Adjacency list of the graph of all the nodes in which we have a collision
    // collision_graph[id1] -> list of all collisions of thread id1 with other threads
    // The adjacency list is kept symmetric such that if there is a collision from id1 to id2
    // it also appears as a collision from id2 to id1.
    // This graph is only read and written by the source thread
    std::vector<std::vector<Collision>>& collision_graph;

    // path from source to goal1 excluding start and end
    std::vector<Node> backward_path;

    // path from source to goal2 excluding start and end
    std::vector<Node> forward_path;

    // Only called by the Source thread to check if there is a path in the collision graph
    // from source to node
    void add_collision(ThreadId collision_source, Collision collision);

    // Only called by the Source thread to check if there is a path in the collision graph
    // from source to node
    void check_collision_path();

    // Called at each iteration of the search by all threads to check messages from other threads
    void check_message_queue();

    // Initialize fringe search list, heuristic and source node cache entry.
    void initialize_fringe_search();

    // Called when a collision happens during the search
    void handle_collision(Node node, ThreadId other);

    // Entry point of all ripple threads
    void entry();

public:

    RippleThread(ThreadId id,
                Map& map, 
                std::vector<std::vector<Collision>>& collision_graph, 
                std::vector<RippleCacheNode>& cache,
                std::vector<concurrent_queue<Message>>& message_queues);

    bool start();
    bool join();
    
    // Setters for the source and goals variables
    void set_source(Node s);
    void set_single_goal(Node g);
    void set_goals(Node g1, Node g2);

    void append_partial_path(std::back_insert_iterator<std::vector<Node>> path_inserter);
};

// Utility class for initializing and invoking ripple search
class RippleSearch {
public:
    Map& map;
    std::vector<concurrent_queue<Message>> message_queues;
    std::vector<RippleCacheNode> cache;
    std::vector<std::vector<Collision>> collision_graph;

    // Initialize search on a map
    RippleSearch(Map& map);

    // Start the search from source to goal, they must be valid nodes
    std::vector<Node> search(Node source, Node goal);
};