#include <memory>
#include <queue>
#include <iostream>

#include "ripple.h"
#include "high_level_path.h"

RippleThread::RippleThread(ThreadId id,
            Map& map, 
            std::vector<std::vector<Collision>>& collision_graph, 
            std::vector<RippleCacheNode>& cache,
            std::vector<concurrent_queue<Message>>& message_queues) :
    id(id),
    thread(nullptr),
    map(map),
    message_queues(message_queues),
    cache(cache),
    collision_graph(collision_graph)
{ }

void RippleThread::set_source(Node s) {
    source = s;
}

void RippleThread::set_single_goal(Node g) {
    goal = g;
    goal_2 = INVALID_NODE;
}

void RippleThread::set_goals(Node g1, Node g2) {
    goal = g1;
    goal_2 = g2;
}

bool RippleThread::start() {
  if (thread == nullptr) {
    thread = std::make_unique<std::thread>(&RippleThread::entry, this);
    return true;
  }
  return false;
}

bool RippleThread::join() {
  if (thread != nullptr) {
    thread->join();
    thread = nullptr;
    return true;
  }
  return false;
}

void RippleThread::add_collision(ThreadId collision_source, Collision collision) {
    collision_graph[collision_source].push_back(collision);
    collision_graph[collision.target].push_back(collision);
}

// Only called by the Source thread to check if there is a path in the collision graph 
// from source to node
void RippleThread::check_collision_path() {
    // Vector of previous nodes in path
    std::pair<ThreadId, Node> prev[NUM_THREADS];
    bool path_found = false;


    // Path find in collision graph
    for(int i = 0; i < NUM_THREADS; i++) {
        prev[i] = std::make_pair(THREAD_NONE, -1);
    }


    // TODO: replace breadth first search with dijkstra or Astar
    std::queue<ThreadId> q;
    q.push(THREAD_SOURCE);

    while(!path_found && !q.empty()) {
        ThreadId u = q.back(); 
        q.pop();
        
        for(size_t i = 0; i < collision_graph[u].size(); i++) {
            auto c = collision_graph[u][i];
            ThreadId v = c.target;
            bool visited = prev[v].first != THREAD_NONE;
            if(visited && map.distance(c.collision_node, goal) < map.distance(prev[v].second, goal)) {
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
        
        // Send message with next source and target to all other threads 
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

        // TODO: Start reconstructing path from source to first collision into the output vector

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

        // TODO: also wait for goal thread
    }
}


// Called at each iteration of the search by all threads to check messages from other threads
void RippleThread::check_message_queue() {
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
            // TODO: save the required state to switch to phase 2
        } break;

        // Only arrives to slave threads if they do not have to work in phase 2 
        case MESSAGE_STOP: {
            // TODO: Exit the thread
        } break;

        default:
            break;
        }
    }

    // The Source thread might need to check collisions
    if(check_collisions) {
        check_collision_path();
    }
}

// Initialize fringe search list, heuristic and source node cache entry.
void RippleThread::initialize_fringe_search() {
    // Initialize fring list with source node only
    fringe_list.clear();
    fringe_list.push_front(source);

    // Set source node cache entry
    FringeNode source_cache;
    source_cache.in_list = true;
    source_cache.g = 0;
    source_cache.parent = source;
    source_cache.visited = true;
    source_cache.list_entry = fringe_list.begin();

    cache[source].node = source_cache;

    // Set the heuristic depending on the numbe of goals
    if(goal_2 == INVALID_NODE) {
        heuristic = [] (RippleThread* self, Node n) {
            return self->map.distance(n, self->goal);
        };
    } else {
        heuristic = [] (RippleThread* self, Node n) {
            return self->map.distance(n, self->goal) + self->map.distance(n, self->goal_2);
        };
    }

    // Set fringe threshold
    flimit = heuristic(this, source);
}


// Called when a collision happens during the search
void RippleThread::handle_collision(Node node, ThreadId other) {
    // TODO: check if the current thread already had a collision with
    // the "other" thread and handle this case
    
    // TODO: For slave threads update the heuristic and check if we are done

    // Message the source thread about the collision, unless we are the source thread
    if(id != THREAD_SOURCE) {
        Message message;
        message.type = MESSAGE_COLLISION;
        message.collision_source = id;
        message.collision.collision_node = node;
        message.collision.target = other;
        message_queues[THREAD_SOURCE].push(message);
    } else {
        Collision collision;
        collision.target = other;
        collision.collision_node = node;
        add_collision(THREAD_SOURCE, collision);
        check_collision_path();
    }
}

void RippleThread::entry() {
    // Initialize fringe search
    initialize_fringe_search();
    
    while (true) {
        // Fringe search step towards G
        bool found = false;
        FringeEntry node = fringe_list.end();

        while (!found && !fringe_list.empty()) {
            int fmin = INT_MAX;
            node = fringe_list.begin();

            do {
                // Check messages from other thread
                check_message_queue();

                // TODO: handle finish and heuristic swap
                if(*node == goal || *node == goal_2) {
                    found = true;
                    break;
                }

                // Load info for the current node
                FringeNode& node_info = cache[*node].node;

                // Compute fringe heuristic for current node
                int g = node_info.g;
                int f = g + heuristic(this, *node);

                // Skip node if fringe heuristic lower than the threshold
                if(f > flimit) {
                    fmin = std::min(fmin, f);
                    node++;
                    continue;
                }

                Point np = map.node_to_point(*node);

                // For each neighbour
                for (int i = 0; i < 4; i++) {
                    Point neigh = Map::neighbour_offsets[i];
                    neigh.x += np.x;
                    neigh.y += np.y;

                    // If the neighbour coordinates are inside the map
                    if (map.in_bounds(neigh)) {
                        Node neighbour = map.point_to_node(neigh);

                        // If it's a wall skip
                        if (!map.get(neigh)) {
                            continue;
                        }
                        
                        // Compute cost of path to s
                        int gs = g + 1;

                        // Check if already owned, otherwise try to acquire
                        ThreadId owner = cache[neighbour].thread.load(std::memory_order_relaxed);
                        if (owner == THREAD_NONE) {
                            if (!cache[neighbour].thread.compare_exchange_strong(owner, id,
                                std::memory_order_seq_cst, std::memory_order_seq_cst)) {
                                
                                // If we failed to acquire the node we need to handle the collision
                                // TODO: enable this once implemented
                                // handle_collision(neighbour, owner);
                                
                                // TODO: For now we are skipping the current node
                                // if it's already visited, should probably go to sleep
                                // and wait for messages instead.
                                continue;
                            }
                        } else {
                            continue;
                        }
                        
                        // Skip neighbour if already visited with a lower cost
                        FringeNode& neighbour_cache = cache[neighbour].node;
                        if (neighbour_cache.visited && gs > neighbour_cache.g) {
                            continue;
                        }

                        // If already in list, remove it
                        if (neighbour_cache.in_list) {
                            fringe_list.erase(neighbour_cache.list_entry);
                            neighbour_cache.in_list = false;
                        }

                        // Insert neighbour in the list right after the current node
                        fringe_list.insert(std::next(node), neighbour);

                        // Update neighbour cache entry
                        neighbour_cache.visited = true;
                        neighbour_cache.g = gs;
                        neighbour_cache.parent = *node;
                        neighbour_cache.list_entry = std::next(node);
                        neighbour_cache.in_list = true;
                    }
                }
                
                // Update current node cache entry
                node_info.in_list = false;

                // Remove current node from the list
                auto tmp = std::next(node);
                fringe_list.erase(node);
                node = tmp;
            } while(node != fringe_list.end());

            // Update fring search threshold with the minimum value present in the new list
            flimit = fmin;
        }

        // Finished the space we could search in

        // Temporary solution for when collision detection is implemented
        /*if (id != THREAD_GOAL) {
          for (Collision c : collision_graph.at(id)) {
            if (c.target == id+1) {
              forward_collision = c;
              break;
            }
          }
        }*/

        // TODO: handle
        if(found) {
            break;
        } else {
            break;
        }

    }
}

ThreadId RippleThread::append_partial_path(std::back_insert_iterator<Path> path_inserter) {
  std::reverse_copy(backward_path.begin(), backward_path.end(), path_inserter);
  *path_inserter = source;
  std::copy(forward_path.begin(), forward_path.end(), path_inserter);

  if (forward_collision.target != THREAD_NONE) {
    *path_inserter = forward_collision.collision_node;
  }
  return forward_collision.target;
}

// Ripple search utilities
RippleSearch::RippleSearch(Map& map):
    map(map),
    cache(map.size())
{
    message_queues.resize(NUM_THREADS);
    collision_graph.resize(NUM_THREADS);
    for(size_t i = 0; i < cache.size(); i++) {
        cache[i].thread.store(THREAD_NONE, std::memory_order_relaxed);
    }
}

Path RippleSearch::search(Node source, Node goal) {
    Path high_level_path = create_high_level_path(map, source, goal);
    if(high_level_path.size() < NUM_THREADS) {
        std::cout << "Failed to find enough nodes for high level path, found: " << high_level_path.size() << std::endl;
        return { };
    }

    //TODO do something smart to choose the best NUM_THREADS points
    while(high_level_path.size() > NUM_THREADS) {
        high_level_path.erase(high_level_path.begin() + high_level_path.size() / 2);
    }
    
    // Initialize first node of cache for each thread.
    // we do this here before starting any thread
    // to avoid race conditions in which a thread tries to 
    // acquire the starting node of another thread.
    for(int i = 0; i < high_level_path.size(); i++) {
        cache[high_level_path[i]].thread.store((ThreadId)i, std::memory_order_seq_cst);
    }

    std::vector<RippleThread*> threads;

    // Start source thread
    {
        auto* source_thread = new RippleThread(THREAD_SOURCE, map, collision_graph, cache, message_queues);
        source_thread->set_source(source);
        source_thread->set_single_goal(goal);

        threads.push_back(source_thread);
    }


    // Start slave thread
    for (int i = 1; i < NUM_THREADS-1; i++) {
        auto* slave_thread = new RippleThread((ThreadId)i, map, collision_graph, cache, message_queues);
        slave_thread->set_source(high_level_path[i]);
        slave_thread->set_goals(high_level_path[i - 1], high_level_path[i + 1]);

        threads.push_back(slave_thread);
    }

    // Start goals thread
    {
        auto* goal_thread = new RippleThread(THREAD_GOAL, map, collision_graph, cache, message_queues);
        goal_thread->set_source(goal);
        goal_thread->set_single_goal(source);

        threads.push_back(goal_thread);
    }

    for(auto& t: threads) {
      t->start();
    }

    // Wait for all threads to finish
    for(auto& t: threads) {
      t->join();
    }

    Path path;
    ThreadId next_thread = THREAD_SOURCE;

    while (next_thread != THREAD_NONE) {
      next_thread = threads.at(next_thread)->append_partial_path(std::back_inserter(path));
    }

    return path.back() == goal ? path : Path();
}
