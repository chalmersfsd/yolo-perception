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
    if (argc < 5 ||
        0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("net_names_file") ||
        0 == commandlineArguments.count("net_cfg_file") || 0 == commandlineArguments.count("net_weights_file"))
    {
        std::cerr << argv[0] << " works with ZED camera directly via USB and posts detections to OD4Session" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid: CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --net_names_file: path to naming file for network" << std::endl;
        std::cerr << "         --net_cfg_file:  path to configuration file of the network" << std::endl;
        std::cerr << "         --net_weights_file: path to trained weights file for network" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=131 --net_names_file=/formula.names " <<
                     "--net_cfg_file=/formula.cfg --net_weights_file=/formula_final.weights --verbose" << std::endl;
    }
    else
    {
        uint16_t cid = static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]));
        std::string network_names = commandlineArguments["net_names_file"];
        std::string network_cfg = commandlineArguments["net_cfg_file"];
        std::string network_weights = commandlineArguments["net_weights_file"];

        cluon::OD4Session od4{cid};
        std::cerr <<  "Start conversation at Opendlv session cid: "<< cid << std::endl;


        send_one_replaceable_object_t<detection_data_t> shared_obj(true);
        detection_data_t detectionResult;
        opendlv::cfsdPerception::Cones8 birdviewData;
        std::thread t_receive = std::thread([&]()
        {
          detectCones(shared_obj, network_names, network_cfg, network_weights);
        });
        while(!detectionResult.exit_flag || !cluon::TerminateHandler::instance().isTerminated.load())
        {
          if (shared_obj.is_object_present())
          {
            detectionResult = shared_obj.receive();
            if (detectionResult.result_vec.size() > 0)
            {
              //Cone ID equals the number of the cone in each frame
              uint32_t coneID = 0;

              show_console_result(detectionResult.result_vec);

              // Calculate 3D coordinates to 2D coordinates and send them out
              cluon::data::TimeStamp now{cluon::time::now()};
              opendlv::logic::perception::ObjectFrameStart startMsg;
              od4.send(startMsg,now,0);

              for(uint32_t n = 0; n < detectionResult.result_vec.size(); n++){
                  //Send cone type
                  opendlv::logic::perception::ObjectType coneType;
                  coneType.type((uint32_t)detectionResult.result_vec[n].obj_id);
                  coneType.objectId(coneID);
                  od4.send(coneType,now,0);

                  //Send cone position
                  opendlv::logic::perception::ObjectPosition conePos;
                  cv::Point2f xy = {0.0, 0.0};
                  //Process 3D data
                  xy = CalculateCone2xy(detectionResult.result_vec[n]);
                  conePos.x(xy.x);
                  conePos.y(xy.y);
                  conePos.objectId(coneID);
                  od4.send(conePos,now,0);

                  coneID++;
                }

                //send Frame End message to mark a frame's end
                opendlv::logic::perception::ObjectFrameEnd endMsg;
                od4.send(endMsg,now,0);
            }
          }
        }
        if (t_receive.joinable()) t_receive.join();

        retCode = 0;
    }

    return retCode;
}
