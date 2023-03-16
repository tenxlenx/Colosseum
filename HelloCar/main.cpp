// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

int main()
{
    using namespace msr::airlib;

    std::cout << "Make sure settings.json has \"SimMode\"=\"Car\" at root. Press Enter to continue." << std::endl;
    std::cin.get();

    // This assumes you are running DroneServer already on the same machine.
    // DroneServer must be running first.
    msr::airlib::CarRpcLibClient client("100.93.151.43");
    //msr::airlib::CarRpcLibClient client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    try {
        client.confirmConnection();

        while (1) {
            vector<ImageRequest> request = { ImageRequest("panorama", ImageType::CubeScene, false, true) };
            const vector<ImageResponse>& response = client.simGetImages(request);
            cv::Mat h_result = cv::imdecode(response[0].image_data_uint8, -1);

            cv::imshow("pano", h_result);
            cv::waitKey(1);
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
        std::cin.get();
    }

    return 0;
}
