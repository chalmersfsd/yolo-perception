#include "cone_detector.hpp"
#include <iostream>
#include <thread>
int main()
{
  send_one_replaceable_object_t<detection_data_t> shared_obj(true);
  detection_data_t data;
  std::thread t_receive = std::thread([&]()
  {
    detectCones(1, NULL, shared_obj);
  });
  while(!data.exit_flag)
  {
    if (shared_obj.is_object_present())
    {
      data = shared_obj.receive();
      if (data.result_vec.size() > 0)
      {
          show_console_result(data.result_vec);
      }
    }
    // else
    //   std::cout << "object in not present" << std::endl;
  }
  if (t_receive.joinable()) t_receive.join();
  return 0;
}
