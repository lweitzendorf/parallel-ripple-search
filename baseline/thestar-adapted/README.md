## A-star baseline

Taken from here: https://github.com/justinhj/astar-algorithm-cpp

Striped the files needed to work for the benchmarking. If we want to benchmark memory management at some point, a memory restrictive function can be introduced.

Run with the make file. Don't change anything in stlastar.h!!!

### Map

Currently only works with hardcoded Bitmap, where bits are 0 and 9. This is due to the astar working with weights. If we decide to go with a Polygraph this can be adapted as well.

### Timing

For now using the timing implementation from Lucas:

For the given 20x20 graph path found in under 0ms without added sleeps. By adding 2 sleeps (before and after finding the path) of 1s each, we reach 2002ms.
