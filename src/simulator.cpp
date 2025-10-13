#include <iostream>
#include <foxglove/foxglove.hpp>
#include <foxglove/server.hpp>
#include <chrono>
#include <csignal>
#include <atomic>
#include <thread>
#include "Quadcopter.h"
#include "Util.h"
#include "Kinematics.h"

using namespace std::chrono_literals;

int main(){
    std::cout << "hi" << std::endl;
    MotorVelocities testV = MotorVelocities(1,1,0,0);
    QCAcceleration test = velocitiesToAccel(testV, Rotation3d(0,0,0));
    QCState testSt = QCState(
        Pose3d(Translation3d(0,0,0), Rotation3d(0,0,0)),
        Pose3d(Translation3d(0,0,0), Rotation3d(0,0,0)), testV, 0);

    QCState next = testSt;
    for(int i = 0; i < 50; i++) {
        next = next.predict(0.01);
        next.getPose().print();
    }


    foxglove::WebSocketServerOptions options;
    auto serverResult = foxglove::WebSocketServer::create(std::move(options));
    if (!serverResult.has_value()) {
        std::cerr << foxglove::strerror(serverResult.error()) << '\n';
        return 1;
    }

    auto server = std::move(serverResult.value());
    auto channel = foxglove::RawChannel::create("/hello", "json").value();
    auto start = std::chrono::steady_clock::now();

    // Log until interrupted
    static std::function<void()> sigint_handler;
    std::atomic_bool done = false;
    sigint_handler = [&] { done = true; };
    std::signal(SIGINT, [](int) {
        if (sigint_handler) {
        sigint_handler();
        }
    });

    while (!done) {
        auto dur = std::chrono::steady_clock::now() - start;
        float elapsed_seconds = std::chrono::duration<float>(dur).count();
        std::string msg = "{\"elapsed\": " + std::to_string(elapsed_seconds) + "}";
        channel.log(reinterpret_cast<const std::byte *>(msg.data()), msg.size());

        std::this_thread::sleep_for(33ms);
    }

    return 0;
}
