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
#include <memory>
#include <mutex>
#include "cone_detector.hpp"

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ||
         (0 == commandlineArguments.count("cid") )) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --name=img.argb --width=640 --height=480 --verbose" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=131 --verbose" << std::endl;
    }
    else {
        //const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        uint16_t cid = static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]));

        // variables for shared memory access
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};

        cluon::OD4Session od4{cid};
        std::cerr <<  "Start conversation at Opendlv session cid: "<< cid << std::endl;

        // Getting a frame from shared memory.
        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Endless loop; end the program by pressing Ctrl-C.
            while (!cluon::TerminateHandler::instance().isTerminated.load()) {
                cv::Mat img;
                opendlv::cfsdPerception::Cones8 coneData;

                // Wait for a notification of a new frame.
                sharedMemory->wait();
                // Lock the shared memory.
                sharedMemory->lock();
                {
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                // Getting the frame processed by CNN tbd

                //coneData = YoloProcessData(img)

                // create timestamp
                cluon::data::TimeStamp now{cluon::time::now()};

                //todo: Sender stamp number
                od4.send(coneData, now, 1902);

            }
        }
        retCode = 0;
    }
    return retCode;

}
