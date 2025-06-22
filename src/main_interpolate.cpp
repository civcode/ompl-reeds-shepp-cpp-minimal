#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <iostream>
#include <vector>

#include "render_module/render_module.hpp" 

namespace ob = ompl::base;

int main()
{
    auto space(std::make_shared<ob::ReedsSheppStateSpace>(1.0));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10); bounds.setHigh(10);
    space->setBounds(bounds);

    ob::ScopedState<> start(space);
    start[0] = 0.0; start[1] = 0.0; start[2] = 0.0;

    ob::ScopedState<> goal(space);
    // goal[0] = 5.0; goal[1] = 5.0; goal[2] = M_PI / 2.0;
    goal[0] = 0.0; goal[1] = 3.0; goal[2] = M_PI/2;
    

    std::vector<ob::State*> path;
    // space->interpolate(start.get(), goal.get(), 20, path);
    int path_length = 50;
    for (int i = 0; i <= path_length; ++i) {
        double t = static_cast<double>(i) / path_length;
        ob::State *s = space->allocState();
        space->interpolate(start.get(), goal.get(), t, s);
        path.push_back(s);
    }
    std::cout << "path.size() = " << path.size() << std::endl;

    std::vector<double> x_pos;
    std::vector<double> y_pos;
    std::vector<double> yaw_pos;
    for (size_t i = 0; i < path.size(); ++i)
    {
        auto* s = path[i]->as<ob::SE2StateSpace::StateType>();
        std::cout << "Pose " << i << ": x=" << s->getX() << ", y=" << s->getY() << ", yaw=" << s->getYaw() << std::endl;
        x_pos.push_back(s->getX());
        y_pos.push_back(s->getY());
        yaw_pos.push_back(s->getYaw());
    }

    // auto get_path = [&]() {

    // }

    RenderModule::Init(980, 720);
    RenderModule::RegisterImGuiCallback([&]() {
        ImGui::Begin("Line Plot");
        ImGui::SetWindowSize(ImVec2(960, 700), ImGuiCond_Once);
        ImGui::SetWindowPos(ImVec2(10, 10), ImGuiCond_Once);
        auto mouse_pos = ImGui::GetMousePos();
        std::cout << "Mouse position: x=" << mouse_pos.x << ", y=" << mouse_pos.y << std::endl;
        if (ImPlot::BeginPlot("Positions", ImVec2(-1, -1), ImPlotFlags_Equal)) {
            auto mouse_pos = ImPlot::GetPlotMousePos();
            std::cout << "Plot Mouse position: x=" << ImPlot::GetPlotMousePos().x << ", y=" << ImPlot::GetPlotMousePos().y << std::endl;
            ImPlot::PlotLine("Position", x_pos.data(), y_pos.data(), x_pos.size());
            ImPlot::EndPlot();
        }
        ImGui::End();
    });
    RenderModule::Run();
    RenderModule::Shutdown(); 

    for (auto* s : path) space->freeState(s);
    return 0;
}