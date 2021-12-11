#include "RippleSearch.h"
#include "RippleThread.h"
#include "HighLevelGraph.h"

#include <iostream>
#include <memory>

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
  Timer timer;

  timer.start();
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

  timer.stop();
  double high = timer.get_microseconds() / 1000.0;

  std::vector<RippleThread *> threads;

  timer.start();
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
    auto *slave_thread = new RippleThread((ThreadId)i, map, collision_graph, cache, message_queues);
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
  timer.stop();

  double search = timer.get_microseconds() / 1000.0;

  timer.start();
  Path<Node> path;
  for (auto &t : threads[THREAD_SOURCE]->get_phase2_thread_path()) {
    auto &p = threads[t]->get_final_path();
    path.insert(path.end(), p.begin(), p.end());
  }
  timer.stop();

  double reconstruct = timer.get_microseconds() / 1000.0;

#if false
  printf("%3.2f path\n", search);
  for(int i = 0; i < NUM_THREADS; i++) {
    printf("%d - %3.2f first - %3.2f second\n", i, threads[i]->time_first, threads[i]->time_second);
  }
#endif

  return path.empty() ? std::nullopt : std::optional<Path<Node>>{path};
}

ThreadId RippleSearch::getOwner(Point p) const {
  return cache[map.point_to_node(p)].thread.load(std::memory_order_relaxed);
}
