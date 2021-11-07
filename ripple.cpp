#include <iostream>

#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <queue>

#include <oneapi/tbb/concurrent_queue.h>
#include <oneapi/tbb/concurrent_vector.h>


using oneapi::tbb::concurrent_queue;
using oneapi::tbb::concurrent_vector;

// TODO: ok??
typedef int Node;
//#define INVALID_NODE -1

enum ThreadId: int8_t {
    THREAD_NONE = -1,
    THREAD_SOURCE = 0,
    THREAD_GOAL,
};

#define NUM_THREADS 16

struct Collision {
    ThreadId target;
    Node collision_node;
};

enum MessageType {
    MESSAGE_PHASE_2,
    MESSAGE_DONE,
    MESSAGE_STOP,
    MESSAGE_COLLISION,
};

struct Message {
    MessageType type;
    union {
        struct {
            Node source;
            Node target;
        };

        struct {
            Collision collision;
            ThreadId collision_source;
        };
    };
};

class RippleThread {
public:
    std::mutex wait_mutex;
    std::condition_variable wait_cv;
    
    // Ref to Graph
    std::vector<char>& graph;
    int graph_width;
    int graph_height;

    // Goal node
    Node goal_node;

    // Ref to vector of node info for fringe search
    std::vector<std::atomic<Node>>& parents;

    // Ref to collision vector
    std::vector<std::atomic<ThreadId>>& thread_marks;

    // Collision graph
    std::vector<std::vector<Collision>>& collision_graph;
    std::mutex collision_mutex;
    std::atomic<bool> phase2_begin;

    // Message queues for sending to other threads
    std::vector<concurrent_queue<Message>>& message_queues;

    // Num of non essential threads
    int num_slaves;

    // My id
    ThreadId id;

public:

    int distance(Node a, Node b) {
        int ax = a % graph_width;
        int ay = a / graph_width;

        int bx = b % graph_width;
        int by = b / graph_width;

        return abs(bx - ax) + abs(by - ay);
    }

    void add_collision(ThreadId collision_source, Collision collision) {
        collision_graph[collision_source].push_back(collision);
        collision_graph[collision.target].push_back(collision);
    }

    void check_collision_path() {
        // Vector of previous nodes in path
        std::pair<ThreadId, Node> prev[NUM_THREADS];
        bool path_found = false;


        // Path find in collision graph
        for(int i = 0; i < NUM_THREADS; i++) {
            prev[i] = std::make_pair(THREAD_NONE, -1);
        }


        // TODO: replace breadth first search with dijkstra's algorithm
        std::queue<ThreadId> q;
        q.push(THREAD_SOURCE);

        while(!path_found && !q.empty()) {
            ThreadId u = q.back(); 
            q.pop();
            
            for(size_t i = 0; i < collision_graph[u].size(); i++) {
                auto c = collision_graph[u][i];
                ThreadId v = c.target;
                bool visited = prev[v].first != THREAD_NONE;
                if(visited && distance(c.collision_node, goal_node) < distance(prev[v].second, goal_node)) {
                    prev[v] = std::make_pair(u, c.collision_node);
                }

                if(!visited) {
                    prev[v] = std::make_pair(u, c.collision_node);
                    q.push(v);
                }

                if(v == THREAD_GOAL) {
                    path_found = true;
                    break;
                }
            }
        }
        
        // TODO split this??

        // If there is a path signal all threads to stop
        if(path_found) {
            // Signal goal thread to stop
            Message m;
            m.type = MESSAGE_PHASE_2;
            message_queues[THREAD_GOAL].push(m);
            
            // For each slave send target and source
            auto p1 = prev[THREAD_GOAL];
            ThreadId u = p1.first;
            Node c1 = p1.second;

            bool works_in_phase2[NUM_THREADS];

            while(u != THREAD_SOURCE) {
                auto p2 = prev[u];
                Node c2 = p2.second;

                m.type = MESSAGE_PHASE_2;
                m.source = c1;
                m.target = c2;

                message_queues[u].push(m);
                works_in_phase2[u] = true;

                u = p2.first;
                c1 = c2;
            }

            // Stop all slaves that don't work in phase 2
            m.type = MESSAGE_STOP;
            int slaves_working = 0;
            for(int i = 2; i < NUM_THREADS; i++) {
                if(!works_in_phase2[i]) {
                    message_queues[i].push(m);
                    slaves_working++;
                }
            }

            // TODO Start reconstructing path from source to first collision into the output vector

            // Wait for all to end
            while(slaves_working) {
                std::unique_lock<std::mutex> lk(wait_mutex);
                wait_cv.wait(lk);

                while(message_queues[THREAD_SOURCE].try_pop(m)) {
                    if(m.type == MESSAGE_DONE) {
                        slaves_working--;
                    }
                }
            }

            // TODO wait for goal thread?
        }
    }

    void entry() {

        // Fringe search step towards G
        while(true) {
            // Check signals
            bool check_collisions = false;
            Message message;
            while(message_queues[id].try_pop(message)) {
                switch (message.type)
                {
                // Only arrives to THREAD_SOURCE
                case MESSAGE_COLLISION: {
                    add_collision(message.collision_source, message.collision);
                    check_collisions = true;
                } break;
                
                // Only arrives to slave threads if they need to switch to phase 2
                case MESSAGE_PHASE_2: {
                    // TODO save the required state to switch to phase 2
                } break;

                // Only arrives to slave threads if they do not have to work in phase 2 
                case MESSAGE_STOP: {
                    return;
                } break;

                default:
                    
                    break;
                }
            }

            if(check_collisions) {
                check_collision_path();
            }

            
            // if fringe search failed and  -> message search failed 


            // Compare and swap into collision vector
            Node u = 0;
            Node parent = 0;
            auto& c = thread_marks[u];

            auto old_id = THREAD_NONE;
            if(c.compare_exchange_strong(old_id, id, std::memory_order_seq_cst, std::memory_order_seq_cst)) {
                //If no collision update parents vector
                parents[u].store(parent, std::memory_order_relaxed);
            } else {
                //If collision push collision to collision graph and signal to SOURCE_THREAD
                if(id != THREAD_SOURCE) {
                    Message m;
                    m.type = MESSAGE_COLLISION;
                    m.collision_source = id;
                    m.collision.collision_node = u;
                    m.collision.target = old_id;
                    message_queues[THREAD_SOURCE].push(m);
                } else {
                    Collision c;
                    c.target = old_id;
                    c.collision_node = u;
                    add_collision(THREAD_SOURCE, c);
                    check_collision_path();
                }
            }
        }
        
#if 0
        int counter = 0;
        while(true) {
            std::unique_lock<std::mutex> lk(m);
            cv.wait(lk);


            int a;
            while(queue.try_pop(a)) {
                counter++;
                std::cout << a << " total: " << counter << std::endl;
            }
        }
#endif
    }
};

class GoalThread {

};

class SlaveThread {

};

int main(int argc, char** argv) {
#if 0
    SourceThread source;
    auto t = std::thread(&SourceThread::entry, &source);

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(2000ms);

    source.queue.push(10);
    source.queue.push(100);

    source.cv.notify_one();
    
    std::this_thread::sleep_for(2000ms);

    source.queue.push(20);
    source.queue.push(300);

    source.cv.notify_one();
    
    

    t.join();
#endif
    return 0;
}
