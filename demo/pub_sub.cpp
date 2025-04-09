#include <node.h>

using namespace Engine;

void cb_a(const std::shared_ptr<int> msg)
{
  std::cout << "sub_a rev " << *msg << std::endl;
}
void cb_b(const std::shared_ptr<int> msg)
{
  std::cout << "sub_b rev " << *msg << std::endl;
}

int main()
{
  std::hash<std::string> hasher;
  Publisher<int> a("int");
  int b = 0;
  Subscription<int> sub_a("int", cb_a);
  Subscription<int> sub_b("int", cb_b);

  while (b < 5)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    a.publish(b);
    b++;
  }
  while (1)
  {
  }
  return 0;
}