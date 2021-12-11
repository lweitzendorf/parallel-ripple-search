#include "RippleSearch.h"
#include "RippleThread.h"
#include "HighLevelGraph.h"
#include "reference/Astar.h"

#include <iostream>
#include <memory>

// Ripple search utilities
RippleSearch::RippleSearch(Map &map, Node source, Node goal)
    : map(map), cache(map.size()), collision_graph(map, goal), source(source),
      goal(goal) {
  message_queues.resize(NUM_THREADS);
  for (auto &entry : cache) {
    entry.thread.store(THREAD_NONE, std::memory_order_relaxed);
  }
}

std::optional<Path<Node>> RippleSearch::search() {
  Path<Node> high_level_path = create_high_level_path(map, source, goal);
  if (high_level_path.size() < NUM_SEARCH_THREADS) {
    std::cout << "Failed to find enough nodes for high level path, found: "
              << high_level_path.size() << std::endl;
    return {};
  }

  // TODO do something smart to choose the best NUM_SEARCH_THREADS points
  while (high_level_path.size() > NUM_SEARCH_THREADS) {
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
    auto *source_thread = new RippleThread(THREAD_SOURCE, map, cache, message_queues);
    source_thread->set_source(source);
    source_thread->set_single_goal(goal);

    threads.push_back(source_thread);
  }

  // Start slave thread
  for (int i = 1; i < NUM_SEARCH_THREADS - 1; i++) {
    auto *slave_thread = new RippleThread((ThreadId)i, map, cache, message_queues);
    slave_thread->set_source(high_level_path[i]);
    slave_thread->set_goals(high_level_path[i - 1], high_level_path[i + 1]);

    threads.push_back(slave_thread);
  }

  // Start goals thread
  {
    auto *goal_thread = new RippleThread(THREAD_GOAL, map, cache, message_queues);
    goal_thread->set_source(goal);
    goal_thread->set_single_goal(source);

    threads.push_back(goal_thread);
  }

  for (auto &t : threads) {
    t->start();
  }

  // HERE if no path exists do something crazy
  auto collision_path = coordinate_threads();

  // Wait for all threads to finish
  for (auto &t : threads) {
    t->join();
  }

  Path<Node> path;
  for (auto &id : collision_path.value_or(Path<ThreadId>())) {
    auto &p = threads[id]->get_final_path();
    if (id == THREAD_SOURCE) {
      std::reverse_copy(p.begin(), p.end(), std::back_inserter(path));
    } else {
      std::copy(p.begin(), p.end(), std::back_inserter(path));
    }
  }

  return path.empty() ? std::nullopt : std::optional<Path<Node>>{path};
}

std::optional<Path<ThreadId>> RippleSearch::coordinate_threads() {
  Message msg;
  int working_threads = NUM_SEARCH_THREADS;

  while (working_threads) {
    if (message_queues[THREAD_COORDINATOR].try_pop(msg)) {
      switch (msg.type) {
        case MESSAGE_COLLISION: {
          LogfNOID("Message - Collision: %d -> %d", msg.collision_source, msg.collision_target);
          add_collision(msg.collision_source, msg.collision_target, msg.collision_node, msg.collision_parent);
          auto maybe_path = check_collision_path();
          if (maybe_path)
            return maybe_path;
        } break;

        case MESSAGE_DONE: {
          working_threads--;
        } break;

        default:
          AssertUnreachable("Coordinator thread should only receive MESSAGE_COLLISION and MESSAGE_DONE messages!");
      }
    }
  }
  return {};
}

void RippleSearch::add_collision(ThreadId src, ThreadId target, Node node, Node parent) {
  collision_graph.add_collision(src, target, node, parent);
}

std::optional<Path<ThreadId>> RippleSearch::check_collision_path() {
  auto maybe_path = a_star_search(collision_graph, THREAD_SOURCE, THREAD_GOAL);

  if(!maybe_path)
    return {};

  // If there is a path signal all threads to stop
  auto found_path = maybe_path.value();

  #if LOG_ENABLED
    LogNOID("Path found:");
    for (auto n : found_path) {
      printf("%d -> ", n);
    }
    printf("||\n");
  #endif

  if (found_path.front() != THREAD_SOURCE)
    AssertUnreachable("I expected the source thread to be the start of the path");

  if (found_path.back() != THREAD_GOAL)
    AssertUnreachable("I expected the goal thread to be the end of the path");

  // Send message with next source and target to all threads
  // that aren't the source / target thread.

  bool works_in_phase2[NUM_SEARCH_THREADS] = {};

  // Workers working
  // For each worker send target and source
  for (int i = 1; i < found_path.size() - 1; i++) {
    ThreadId thread = found_path[i];
    auto collision_with_prev = collision_graph.get_collision(found_path[i - 1], found_path[i]);
    // TODO we could make this /slightly/ faster by holding on to these values
    //      as the 'collision_with_prev' at a later point. (same for the
    //      collision_path values)
    auto collision_with_next = collision_graph.get_collision(found_path[i], found_path[i + 1]);

    Message msg {
      MESSAGE_PHASE_2,
      { .source = collision_with_next.node,
        .target = collision_with_prev.node }
    };

    // Set ownership of goal node to the current thread
    cache[msg.target].thread.store(thread, std::memory_order_seq_cst);
    message_queues[thread].push(msg);
    works_in_phase2[thread] = true;

    LogfNOID("Phase2: WORKER %d <- %d -> %d", found_path[i - 1], thread, found_path[i + 1]);
  }

  // WORKERS STOPPED
  // Stop all workers that don't work in phase 2
  Message msg { .type = MESSAGE_STOP };
  for (int i = 1; i < NUM_SEARCH_THREADS - 1; i++)
    if (!works_in_phase2[i])
      message_queues[i].push(msg);

  // NOTE:
  // The order between source and goal here is important when the only
  // collision is the one between source and goal. In that case we
  // first store into current the node that source starts reversing from
  // and then update the node cache so that goal is able to reverse his.
  // This should probably be made more robust once we handle collisions
  // better.

  // SOURCE
  // Start reconstructing path from source to first
  Collision first_collision = collision_graph.get_collision(THREAD_SOURCE, found_path[1]);
  Node current = first_collision.parent;
  if (first_collision.target == THREAD_SOURCE)
    current = cache[first_collision.node].node.parent;

  // GOAL
  // Update the cache, so that the parent of the last collision is always a
  // node that was discovered by goal thread.
  ThreadId second_last = found_path[found_path.size() - 2];
  Collision last_collision = collision_graph.get_collision(second_last, THREAD_GOAL);

  if (last_collision.target != THREAD_GOAL) {
    cache[last_collision.node].thread.store(THREAD_GOAL,std::memory_order_seq_cst);
    cache[last_collision.node].node.parent = last_collision.parent;
  } else
    assert(cache[last_collision.node].thread.load(std::memory_order_seq_cst) == THREAD_GOAL);

  msg = Message { .type = MESSAGE_PHASE_2, .final_node = first_collision.node };
  message_queues[THREAD_SOURCE].push(msg);

  msg = Message { .type = MESSAGE_PHASE_2, .final_node = last_collision.node };
  message_queues[THREAD_GOAL].push(msg);

  return found_path;
}

ThreadId RippleSearch::get_owner(Point p) const {
  return cache[map.point_to_node(p)].thread.load(std::memory_order_relaxed);
}
