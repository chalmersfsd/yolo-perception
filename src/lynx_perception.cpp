/*
 * Copyright (C) 2019   Felix Hörnschemeyer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "lynx_perception.hpp"

#include <cstdint>
#include <iostream>
#include <thread>
#include <memory>
#include <mutex>
#include "cone_detector.hpp"

int32_t main(int32_t argc, char **argv) {
    int32_t retCode = 1;

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0 == commandlineArguments.count("cid") )
    {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --name=img.argb --width=640 --height=480 --verbose" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=131 --verbose" << std::endl;
    }
    else
    {
        //const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        uint16_t cid = static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]));

        cluon::OD4Session od4{cid};
        std::cerr <<  "Start conversation at Opendlv session cid: "<< cid << std::endl;


        send_one_replaceable_object_t<detection_data_t> shared_obj(true);
        detection_data_t detectionResult;
        opendlv::cfsdPerception::Cones8 birdviewData;
        std::thread t_receive = std::thread([&]()
        {
          detectCones(1, NULL, shared_obj);
        });
        while(!detectionResult.exit_flag || !cluon::TerminateHandler::instance().isTerminated.load())
        {
          if (shared_obj.is_object_present())
          {
            detectionResult = shared_obj.receive();
            if (detectionResult.result_vec.size() > 0)
            {
                show_console_result(detectionResult.result_vec);              
                CalculateCone2xy(detectionResult.result_vec);
            }
          }
        }
        if (t_receive.joinable()) t_receive.join();

        retCode = 0;
    }
    return retCode;

}
