#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
//#include <SpatialDetectionConverterEx.hpp>


// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

#include "jsoncpp/json/json.h"

std::tuple<dai::Pipeline, int, int, int, int> createPipeline(bool lrcheck, 
                                                   bool extended, 
                                                   bool syncNN, 
                                                   bool subpixel, 
                                                   std::string nnPath, 
                                                   std::string nnConfigPath, 
                                                   int confidence, 
                                                   int LRchecktresh, 
                                                   std::string resolution,
                                                   bool publish_grayscale_image, 
                                                   bool publish_depth_image) {

    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto monoLeftCam = pipeline.create<dai::node::MonoCamera>();
    auto monoRightCam = pipeline.create<dai::node::MonoCamera>();
    auto stereoDepth = pipeline.create<dai::node::StereoDepth>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();

    std::cout << "publish_grayscale_image = " << publish_grayscale_image << std::endl;

    std::shared_ptr<dai::node::XLinkOut> xoutLeft(nullptr);
    std::shared_ptr<dai::node::XLinkOut> xoutRight(nullptr);

    if(publish_grayscale_image){
        xoutLeft = pipeline.create<dai::node::XLinkOut>();
        xoutRight = pipeline.create<dai::node::XLinkOut>();
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
    }
    auto xoutColor = pipeline.create<dai::node::XLinkOut>();
    xoutColor->setStreamName("color");

    auto xoutColorDetections = pipeline.create<dai::node::XLinkOut>();
    xoutColorDetections->setStreamName("color_detections");


    std::shared_ptr<dai::node::XLinkOut> xoutDepth(nullptr);   
    if(publish_depth_image){
        xoutDepth = pipeline.create<dai::node::XLinkOut>();
        xoutDepth->setStreamName("depth");
    }
    auto xoutDetections = pipeline.create<dai::node::XLinkOut>();
    xoutDetections->setStreamName("detections");


    int width, height;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeftCam->setResolution(monoResolution);
    monoLeftCam->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRightCam->setResolution(monoResolution);
    monoRightCam->setBoardSocket(dai::CameraBoardSocket::CAM_C);


    int nnInputWidth = 416;
    int nnInputHeight = 416;
    int numClasses = 0;
    int coordinateSize = 4;
    float confThreshold = 0.5f;
    float iouThreshold = 0.5f;
    std::vector<float> anchors;
    std::map<std::string, std::vector<int>> anchorMasks;

    // StereoDepth
    stereoDepth->initialConfig.setConfidenceThreshold(confidence);
    stereoDepth->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereoDepth->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereoDepth->setLeftRightCheck(lrcheck);
    stereoDepth->setExtendedDisparity(extended);
    stereoDepth->setSubpixel(subpixel);
    stereoDepth->setDepthAlign(dai::CameraBoardSocket::CAM_A); //added by gerard
    //stereoDepth->Input.setBlocking(false);

    {
        std::ifstream file(nnConfigPath);
        Json::Reader reader;
        Json::Value completeJsonData;
        if(!reader.parse(file, completeJsonData)) {
            throw std::runtime_error("Unable to parse nnConfig JSON: " + nnConfigPath);
        }

        if(completeJsonData.isMember("nn_config")) {
            const auto& nnConfigJson = completeJsonData["nn_config"];
            if(nnConfigJson.isMember("input_size")) {
                auto inputSize = nnConfigJson["input_size"].asString();
                auto delimPos = inputSize.find('x');
                if(delimPos != std::string::npos) {
                    nnInputWidth = std::stoi(inputSize.substr(0, delimPos));
                    nnInputHeight = std::stoi(inputSize.substr(delimPos + 1));
                }
            }

            const auto& metadata = nnConfigJson["NN_specific_metadata"];
            if(metadata.isMember("classes")) {
                if(metadata["classes"].isString()) numClasses = std::stoi(metadata["classes"].asString());
                else numClasses = metadata["classes"].asInt();
            }
            if(metadata.isMember("coordinates")) {
                if(metadata["coordinates"].isString()) coordinateSize = std::stoi(metadata["coordinates"].asString());
                else coordinateSize = metadata["coordinates"].asInt();
            }
            if(metadata.isMember("confidence_threshold")) confThreshold = metadata["confidence_threshold"].asFloat();
            if(metadata.isMember("iou_threshold")) iouThreshold = metadata["iou_threshold"].asFloat();

            Json::Value anchorsJson = metadata["anchors"];
            for(int i = 0; i < anchorsJson.size(); i++) {
                if(anchorsJson[i].isString()) anchors.push_back(std::stof(anchorsJson[i].asString()));
                else anchors.push_back(anchorsJson[i].asFloat());
            }

            Json::Value masksJson = metadata["anchor_masks"];
            for(const auto& id : masksJson.getMemberNames()) {
                Json::Value maskMembersJson = masksJson[id];
                std::vector<int> maskValues;
                for(int i = 0; i < maskMembersJson.size(); i++) {
                    if(maskMembersJson[i].isString()) maskValues.push_back(std::stoi(maskMembersJson[i].asString()));
                    else maskValues.push_back(maskMembersJson[i].asInt());
                }
                anchorMasks[id] = maskValues;
            }
        } else if(completeJsonData.isMember("model")) {
            const auto& modelJson = completeJsonData["model"];
            if(modelJson.isMember("inputs") && modelJson["inputs"].size() > 0) {
                const auto& shape = modelJson["inputs"][0]["shape"];
                if(shape.size() >= 4) {
                    nnInputHeight = shape[2].asInt();
                    nnInputWidth = shape[3].asInt();
                }
            }

            if(modelJson.isMember("heads") && modelJson["heads"].size() > 0) {
                const auto& metadata = modelJson["heads"][0]["metadata"];
                if(metadata.isMember("n_classes")) numClasses = metadata["n_classes"].asInt();
                if(metadata.isMember("conf_threshold")) confThreshold = metadata["conf_threshold"].asFloat();
                if(metadata.isMember("iou_threshold")) iouThreshold = metadata["iou_threshold"].asFloat();

                if(metadata.isMember("anchors") && metadata["anchors"].isArray()) {
                    for(int i = 0; i < metadata["anchors"].size(); i++) {
                        anchors.push_back(metadata["anchors"][i].asFloat());
                    }
                }
            }
        } else {
            throw std::runtime_error("Unsupported nnConfig JSON schema: " + nnConfigPath);
        }

        if(numClasses <= 0) {
            throw std::runtime_error("Invalid class count in nnConfig: " + nnConfigPath);
        }
    }

    colorCam->setPreviewSize(nnInputWidth, nnInputHeight);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(confThreshold);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    spatialDetectionNetwork->setNumClasses(numClasses);
    spatialDetectionNetwork->setCoordinateSize(coordinateSize);
    if(!anchors.empty()) spatialDetectionNetwork->setAnchors(anchors);
    if(!anchorMasks.empty()) spatialDetectionNetwork->setAnchorMasks(anchorMasks);
    spatialDetectionNetwork->setIouThreshold(iouThreshold);
    spatialDetectionNetwork->setSpatialCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::MIN);


    // Link plugins CAM -> STEREO -> XLINK
    monoLeftCam->out.link(stereoDepth->left);
    monoRightCam->out.link(stereoDepth->right);

    if(publish_grayscale_image){
        stereoDepth->rectifiedLeft.link(xoutLeft->input);
        stereoDepth->rectifiedRight.link(xoutRight->input);
    }
    //colorCam->preview.link(xoutColor->input);
    stereoDepth->depth.link(xoutDepth->input);

    syncNN = true; // added by gerard

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(spatialDetectionNetwork->input);
#if 0
    if(syncNN)
        spatialDetectionNetwork->passthrough.link(xoutColor->input);
    else
        colorCam->preview.link(xoutColor->input);
#endif
    spatialDetectionNetwork->passthrough.link(xoutColorDetections->input);
    colorCam->video.link(xoutColor->input);

    //colorCam->preview.link(xoutColor->input);
    spatialDetectionNetwork->out.link(xoutDetections->input);

    stereoDepth->depth.link(spatialDetectionNetwork->inputDepth);
    //stereoDepth->depth.link(xoutDepth->input);
    if(publish_depth_image){
        spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
    }
    return std::make_tuple(pipeline, width, height, nnInputWidth, nnInputHeight);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yolo_spatial_detector_node");

    std::string resourceBaseFolder, nnPath, nnConfigPath;
    std::string camera_param_uri;
    std::string nnName, nnConfig;  // Set your blob name for the model here
    //std::string monoResolution = "400p";

    std::string tfPrefix, mode, monoResolution;
    bool syncNN;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence, LRchecktresh;
    int monoWidth, monoHeight;
    int nnInputWidth, nnInputHeight;
    dai::Pipeline pipeline;
    bool publish_grayscale_image, publish_depth_image;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", camera_param_uri);
    node->declare_parameter("sync_nn", true);
    node->declare_parameter("nnConfig", "");
    node->declare_parameter("subpixel", true);
    node->declare_parameter("nnName", "");
    node->declare_parameter("confidence", confidence);
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("LRchecktresh", LRchecktresh);
    node->declare_parameter("monoResolution", monoResolution);
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("extended", false);
    node->declare_parameter("publish_grayscale_image", true);
    node->declare_parameter("publish_depth_image", true);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", camera_param_uri);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);
    node->get_parameter("nnConfig", nnConfig);

    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("extended", extended);

    node->get_parameter("publish_grayscale_image", publish_grayscale_image);
    node->get_parameter("publish_depth_image", publish_depth_image);


    // forse true !!! for testing
    //publish_grayscale_image = true;
    //publish_depth_image = true;

    //std::cout << resourceBaseFolder << std::endl;

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }

    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {

        node->get_parameter("nnName", nnName);
        std::cout << "nn_name: " << nnName << std::endl;
    }

    nnPath = resourceBaseFolder + "/" + nnName;
    //std::cout << " Path nn: " << nnPath <<std::endl;
    
    nnConfigPath = resourceBaseFolder + "/" + nnConfig;
    //std::cout << " Path config: " << nnConfigPath <<std::endl;
    
    std::tie(pipeline, monoWidth, monoHeight, nnInputWidth, nnInputHeight) = createPipeline(lrcheck, 
                                                                                              extended, 
                                                                                              syncNN, 
                                                                                              subpixel, 
                                                                                              nnPath, 
                                                                                              nnConfigPath, //
                                                                                              confidence, 
                                                                                              LRchecktresh, 
                                                                                              monoResolution,
                                                                                              publish_grayscale_image, 
                                                                                              publish_depth_image);
    
    dai::Device device(pipeline);

    auto colorQueue = device.getOutputQueue("color", 30, false);
    auto colorDetectionsQueue = device.getOutputQueue("color_detections", 30, false);
    auto detectionQueue = device.getOutputQueue("detections", 30, false);

    std::shared_ptr<dai::DataOutputQueue> stereoDepthQueue(nullptr);
    if(publish_depth_image){
        stereoDepthQueue = device.getOutputQueue("depth", 30, false);
    }

    std::shared_ptr<dai::DataOutputQueue> leftQueue(nullptr);
    std::shared_ptr<dai::DataOutputQueue> rightQueue(nullptr);
    if(publish_grayscale_image){
        leftQueue = device.getOutputQueue("left", 30, false);
        rightQueue = device.getOutputQueue("right", 30, false);
    }

    auto calibrationHandler = device.readCalibration();
    
    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    //dai::rosBridge::ImageConverter *leftconverter;
    //dai::rosBridge::ImageConverter::calibrationToCameraInfo *leftCameraInfo;
    dai::rosBridge::ImageConverter leftconverter(tfPrefix + "_left_camera_optical_frame", true);
    auto leftCameraInfo = leftconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> *leftPublish;

    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> *rightPublish;

    if(publish_grayscale_image){
        leftPublish = new dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>(
            leftQueue,
            node,
            std::string("left/image_rect"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, leftconverter, std::placeholders::_1, std::placeholders::_2),
            30,
            leftCameraInfo,
            "left");
        leftPublish->addPublisherCallback();

        rightPublish = new dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>(
            rightQueue,
            node,
            std::string("right/image_rect"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
            30,
            rightCameraInfo,
            "right");
        rightPublish->addPublisherCallback();
    }

    dai::rosBridge::ImageConverter color_converter(tfPrefix + "_rgb_camera_optical_frame", true);
//    auto colorCameraInfo = color_converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C,-1,-1);//416, 416);
    auto colorCameraInfo = color_converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, -1, -1);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> colorPublish(
        colorQueue,
        node,
        std::string("color/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &color_converter, std::placeholders::_1, std::placeholders::_2),
        30,
        colorCameraInfo,
        "color");
    colorPublish.addPublisherCallback();

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> colorDetectionsPublish(
        colorDetectionsQueue,
        node,
        std::string("color/detections"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &color_converter, std::placeholders::_1, std::placeholders::_2),
        30,
        colorCameraInfo,
        "color");
    colorDetectionsPublish.addPublisherCallback();

//    auto depthconverter = rightconverter;//color_converter;
    auto depthconverter = color_converter;
//    auto depthCameraInfo = rightCameraInfo;
    auto depthCameraInfo =  depthconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, -1,-1);//416, 416);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> *depthPublish;
    if(publish_depth_image){
        depthPublish = new dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>(
            stereoDepthQueue,
            node,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                        &depthconverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                        std::placeholders::_1,
                        std::placeholders::_2),
            30,
            depthCameraInfo,
            "stereo");
        depthPublish->addPublisherCallback();
    }

    dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", nnInputWidth, nnInputHeight, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        detectionQueue,
        node,
        std::string("color/yolov4_spatial_detections"),
        std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    detectionPublish.addPublisherCallback();

    rclcpp::spin(node);

    return 0;
}
