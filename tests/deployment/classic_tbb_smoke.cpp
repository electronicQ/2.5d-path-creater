#include <tbb/parallel_for.h>
#include <tbb/task_scheduler_init.h>
#include <vector>
int main() {
  tbb::task_scheduler_init scheduler(2);
  std::vector<int> values(100, 0);
  tbb::parallel_for(0, 100, [&](int index) { values[index] = index; });
  return values[99] == 99 ? 0 : 1;
}
