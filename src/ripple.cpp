#include <iostream>
#include <memory>
#include <queue>

#include "Astar.h"
#include "high_level_path.h"
#include "ripple.h"

RippleThread::RippleThread(
    ThreadId id, Map &map, CollisionGraph &collision_graph,
    std::vector<RippleCacheNode> &cache,
    std::vector<concurrent_queue<Message>> &message_queues)
    : id(id), thread(nullptr), map(map), message_queues(message_queues),
      cache(cache), collision_graph(collision_graph) {}

void RippleThread::set_source(Node s) { source = s; }

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

void RippleThread::add_collision(ThreadId collision_source,
                                 Collision collision) {
  if (id != THREAD_SOURCE)
    throw std::invalid_argument("add_collision called on non-source thread");
  collision_graph.add_collision(collision_source, collision);
}

// Only called by the Source thread to check if there is a path in the collision
// graph from source to node
void RippleThread::check_collision_path() {
  if (id != THREAD_SOURCE)
    throw std::invalid_argument(
        "check_collision_graph called on non-source thread");

  auto maybe_path =
      a_star_search_gen(collision_graph, THREAD_SOURCE, THREAD_GOAL);

  bool path_found = maybe_path.has_value();

  // If there is a path signal all threads to stop
  if (path_found) {
    Path<ThreadId> found_path =
        reconstruct_path_gen(THREAD_SOURCE, THREAD_GOAL, maybe_path.value());

    // Signal goal thread to stop
    Message msg;
    msg.type = MESSAGE_PHASE_2;
    message_queues[THREAD_GOAL].push(msg);

    // For each slave send target and source

    bool works_in_phase2[NUM_THREADS];

    if (found_path.front() != THREAD_SOURCE)
      throw std::invalid_argument(
          "I expected the source thread to be the start of the path");
    if (found_path.back() != THREAD_GOAL)
      throw std::invalid_argument(
          "I expected the goal thread to be the end of the path");

    // Send message with next source and target to all threads
    // that aren't the source / target thread.
    for (int i = 1; i < found_path.size() - 1; i++) {
      ThreadId thread = found_path[i];
      auto collision_with_prev =
          collision_graph.collision_point(found_path[i - 1], found_path[i]);
      auto collision_with_next =
          collision_graph.collision_point(found_path[i], found_path[i + 1]);

      msg.type = MESSAGE_PHASE_2;
      msg.source = collision_with_prev;
      msg.target = collision_with_next;

      message_queues[thread].push(msg);
      works_in_phase2[thread] = true;
    }

    // Stop all slaves that don't work in phase 2
    msg.type = MESSAGE_STOP;
    int slaves_working = 0;
    for (int i = 2; i < NUM_THREADS; i++) {
      if (!works_in_phase2[i]) {
        message_queues[i].push(msg);
        slaves_working++;
      }
    }

    // TODO: Start reconstructing path from source to first collision into the
    // output vector

    // Wait for all to end
    while (slaves_working) {
      std::unique_lock<std::mutex> lk(wait_mutex);
      wait_cv.wait(lk);

      while (message_queues[THREAD_SOURCE].try_pop(msg)) {
        if (msg.type == MESSAGE_DONE) {
          slaves_working--;
        }
      }
    }

    // TODO: also wait for goal thread
  }
}

// Called at each iteration of the search by all threads to check messages from
// other threads
bool RippleThread::check_message_queue() {
  // Check signals
  bool check_collisions = false;
  bool start_phase_2 = false;
  Message message;
  while (message_queues[id].try_pop(message)) {
    switch (message.type) {
    // Only arrives to THREAD_SOURCE
    case MESSAGE_COLLISION: {
      add_collision(message.collision_source, message.collision);
      check_collisions = true;
    } break;

      // Only arrives to slave threads if they need to switch to phase 2
    case MESSAGE_PHASE_2: {
      if (id == THREAD_GOAL)
        ; // TODO
      else if (id == THREAD_SOURCE)
        ; // TODO
      else
        reset_for_phase_2(message.source, message.target);
      start_phase_2 = true;
    } break;

      // Only arrives to slave threads if they do not have to work in phase 2
    case MESSAGE_STOP:
      std::exit(0); // TODO: Exit the thread
    }
  }

  // The Source thread might need to check collisions
  if (check_collisions) {
    check_collision_path();
  }
  return start_phase_2;
}

void RippleThread::reset_for_phase_2(Node source, Node target) {
  set_source(source);
  set_single_goal(target);
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
  if (goal_2 == INVALID_NODE) {
    heuristic = [](RippleThread *self, Node n) {
      return self->map.distance(n, self->goal);
    };
  } else {
    heuristic = [](RippleThread *self, Node n) {
      return std::min(self->map.distance(n, self->goal),
                      self->map.distance(n, self->goal_2));
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

  // Message the source thread about the collision, unless we are the source
  // thread
  if (id != THREAD_SOURCE) {
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

bool RippleThread::should_reset() {
  // HACK this function in theory should return
  //      whether or not the thread should reset for phase 2.
  //      Once we start doing phase two this should return the
  //      value of the checked message queue.
  return check_message_queue();
}

void RippleThread::entry() {

// NOTE this is terrible style, however, as structured, there is no
//      alternative to break of of three nested loops to reset the search.
//      Gavin would like to prepose a rewrite that would better handle
//      this case.
//      - use continuations (best semantics | possibly slow in c++ impl)
//      - use goto (worst semantics | probably fast requires less code rewrite)
//      - alternative: decouple these loops and stop relying on internal state
//      so much
reset:

  initialize_fringe_search();

  while (true) {
    // Fringe search step towards G
    bool found = false;
    FringeEntry node = fringe_list.end();

    while (!found && !fringe_list.empty()) {
      int fmin = INT_MAX;
      node = fringe_list.begin();

      do {
        if (should_reset())
          goto reset;

        // TODO: handle finish and heuristic swap
        if (*node == goal || *node == goal_2) {
          found = true;
          break;
        }

        // Load info for the current node
        FringeNode &node_info = cache[*node].node;

        // Compute fringe heuristic for current node
        int g = node_info.g;
        int f = g + heuristic(this, *node);

        // Skip node if fringe heuristic lower than the threshold
        if (f > flimit) {
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
            ThreadId owner =
                cache[neighbour].thread.load(std::memory_order_relaxed);
            if (owner == THREAD_NONE) {
              if (!cache[neighbour].thread.compare_exchange_strong(
                      owner, id, std::memory_order_seq_cst,
                      std::memory_order_seq_cst)) {

                // If we failed to acquire the node we need to handle the
                // collision
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
            FringeNode &neighbour_cache = cache[neighbour].node;
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
      } while (node != fringe_list.end());

      // Update fring search threshold with the minimum value present in the new
      // list
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
    if (found) {
      break;
    } else {
      break;
    }
  }
}

ThreadId RippleThread::append_partial_path(
    std::back_insert_iterator<Path<Node>> path_inserter) {
  std::reverse_copy(backward_path.begin(), backward_path.end(), path_inserter);
  *path_inserter = source;
  std::copy(forward_path.begin(), forward_path.end(), path_inserter);

  if (forward_collision.target != THREAD_NONE) {
    *path_inserter = forward_collision.collision_node;
  }
  return forward_collision.target;
}

// Ripple search utilities
RippleSearch::RippleSearch(Map &map, Node source, Node goal)
    : map(map), cache(map.size()), collision_graph(map, goal), source(source),
      goal(goal) {
  message_queues.resize(NUM_THREADS);
  for (size_t i = 0; i < cache.size(); i++) {
    cache[i].thread.store(THREAD_NONE, std::memory_order_relaxed);
  }
}

std::optional<Path<Node>> RippleSearch::search() {
  Path<Node> high_level_path = create_high_level_path(map, source, goal);
  if (high_level_path.size() < NUM_THREADS) {
    std::cout << "Failed to find enough nodes for high level path, found: "
              << high_level_path.size() << std::endl;
    return {};
  }

  // TODO do something smart to choose the best NUM_THREADS points
  while (high_level_path.size() > NUM_THREADS) {
    high_level_path.erase(high_level_path.begin() + high_level_path.size() / 2);
  }

  // Initialize first node of cache for each thread.
  // we do this here before starting any thread
  // to avoid race conditions in which a thread tries to
  // acquire the starting node of another thread.
  for (int i = 0; i < high_level_path.size(); i++) {
    cache[high_level_path[i]].thread.store((ThreadId)i,
                                           std::memory_order_seq_cst);
  }

  std::vector<RippleThread *> threads;

  // Start source thread
  {
    auto *source_thread = new RippleThread(THREAD_SOURCE, map, collision_graph,
                                           cache, message_queues);
    source_thread->set_source(source);
    source_thread->set_single_goal(goal);

    threads.push_back(source_thread);
  }

  // Start slave thread
  for (int i = 1; i < NUM_THREADS - 1; i++) {
    auto *slave_thread = new RippleThread((ThreadId)i, map, collision_graph,
                                          cache, message_queues);
    slave_thread->set_source(high_level_path[i]);
    slave_thread->set_goals(high_level_path[i - 1], high_level_path[i + 1]);

    threads.push_back(slave_thread);
  }

  // Start goals thread
  {
    auto *goal_thread = new RippleThread(THREAD_GOAL, map, collision_graph,
                                         cache, message_queues);
    goal_thread->set_source(goal);
    goal_thread->set_single_goal(source);

    threads.push_back(goal_thread);
  }

  for (auto &t : threads) {
    t->start();
  }

  // Wait for all threads to finish
  for (auto &t : threads) {
    t->join();
  }

  Path<Node> path;
  ThreadId next_thread = THREAD_SOURCE;

  while (next_thread != THREAD_NONE) {
    next_thread =
        threads.at(next_thread)->append_partial_path(std::back_inserter(path));
  }

  return path.back() == goal ? std::optional<Path<Node>>{path} : std::nullopt;
}
