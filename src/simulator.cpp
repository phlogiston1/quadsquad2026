#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/schemas.hpp>
#include <foxglove/server.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <iostream>
#include <thread>
#include "Quadcopter.h"
#include "Util.h"
#include "Kinematics.h"

using namespace std::chrono_literals;

bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

int main(){
    std::cout << "hi" << std::endl;
    MotorVelocities testV = MotorVelocities(3.1,3,3,3);
    QCAcceleration test = velocitiesToAccel(testV, Rotation3d(0,0,0));
    QCState testSt = QCState(
        Pose3d(Translation3d(0,0,0), Rotation3d(0,0,0)),
        Pose3d(Translation3d(0,0,0), Rotation3d(0,0,0)), testV, 0);

    QCState next = testSt;
    for(int i = 0; i < 50; i++) {
        next = next.predict(0.01);
        next.getPose().print();
    }

    static std::function<void()> sigint_handler;

    std::signal(SIGINT, [](int) {
        if (sigint_handler) {
        sigint_handler();
        }
    });


    //connect to server
    foxglove::WebSocketServerOptions options;
    auto serverResult = foxglove::WebSocketServer::create(std::move(options));
    if (!serverResult.has_value()) {
        std::cerr << foxglove::strerror(serverResult.error()) << '\n';
        return 1;
    }

    auto server = std::move(serverResult.value());

    // Create a schema for a JSON channel for logging {size: number}
    foxglove::Schema schema;
    schema.encoding = "jsonschema";
    std::string schema_data = R"({
            "type": "object",
            "properties": {
            "size": { "type": "number" }
            }
        })";
    schema.data = reinterpret_cast<const std::byte*>(schema_data.data());
    schema.data_len = schema_data.size();
    auto channel_result = foxglove::RawChannel::create("/size", "json", std::move(schema));
    if (!channel_result.has_value()) {
        std::cerr << "Failed to create channel: " << foxglove::strerror(channel_result.error()) << '\n';
        return 1;
    }
    auto size_channel = std::move(channel_result.value());

    // Create a SceneUpdateChannel for logging changes to a 3d scene
    auto scene_channel_result = foxglove::schemas::SceneUpdateChannel::create("/scene");
    if (!scene_channel_result.has_value()) {
        std::cerr << "Failed to create scene channel: "
                << foxglove::strerror(scene_channel_result.error()) << '\n';
        return 1;
    }
    auto scene_channel = std::move(scene_channel_result.value());


    std::atomic_bool done = false;
    sigint_handler = [&] {
        done = true;
    };

    int numIters = 0;
    double last = 0;

    while (!done) {
        if(numIters > 500) {
            next = testSt;
            numIters = 0;
        }

        auto now = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        if(numIters > 0){
            next = next.predict(now - last);
        }
        last = now;
        auto rotation = next.getPose().getRotation();
        // rotation.rotateBy(Rotation3d(0,0,M_PI)); //rotate 180 degrees around z axis to match foxglove coordinate system
        auto quaternion = rotation.toQuaternion();

        double size = std::abs(std::sin(now)) + 1.0;
        std::string msg = "{\"size\": " + std::to_string(size) + "}";
        size_channel.log(reinterpret_cast<const std::byte*>(msg.data()), msg.size());

        foxglove::schemas::CubePrimitive cube;
        cube.size = foxglove::schemas::Vector3{1, 1, 0.3};
        cube.color = foxglove::schemas::Color{1, 1, 1, 1};
        cube.pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{-0.1*next.getPose().getX(), -0.1*next.getPose().getY(), -0.1*next.getPose().getZ()}, foxglove::schemas::Quaternion{quaternion.x,quaternion.y,quaternion.z,quaternion.w}};

        foxglove::schemas::SceneEntity entity;
        entity.id = "box";
        entity.cubes.push_back(cube);

        foxglove::schemas::SceneUpdate scene_update;
        scene_update.entities.push_back(entity);

        scene_channel.log(scene_update);

        numIters++;
        std::this_thread::sleep_for(33ms);
    }

    return 0;
}
