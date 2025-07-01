#include <iostream>
#include <vector>

#include "reeds_shepp/reeds_shepp_wrapper.hpp"
#include "render_module/render_module.hpp" 

int main()
{
    float turning_radius = 4.0;
    // auto space(std::make_shared<ob::ReedsSheppStateSpace>(4.0));
    auto space = std::make_shared<ReedsSheppWrapper>(turning_radius);
    /* bound are NOT required for Reeds-Shepp solutiong */
    // ob::RealVectorBounds bounds(2);
    // bounds.setLow(-10); bounds.setHigh(10);
    // space->setBounds(bounds);
     

    ob::ScopedState<> start(space);
    start[0] = 0.0; start[1] = 0.0; start[2] = 0.0;

    ob::ScopedState<> goal(space);
    // goal[0] = 5.0; goal[1] = 5.0; goal[2] = M_PI / 2.0;
    goal[0] = 0.0; goal[1] = 3.0; goal[2] = M_PI/2;
    double goal_yaw = goal[2];
    
    struct data {
        std::vector<double> x_pos;
        std::vector<double> y_pos;
        std::vector<double> yaw_pos;
    };

    auto get_path = [&](ImPlotPoint point) -> data {
        goal[0] = point.x;
        goal[1] = point.y;

        /* Generate and interpolate Reeds-Shepp path */
        std::vector<ob::State*> path;
        int path_length = 50;
        for (int i = 0; i <= path_length; ++i) {
            double t = static_cast<double>(i) / path_length;
            ob::State *s = space->allocState();
            space->interpolate(start.get(), goal.get(), t, s);
            path.push_back(s);
        }
        // std::cout << "path.size() = " << path.size() << " elements" << std::endl;

        /* Copy data for plotting */
        std::vector<double> x_pos;
        std::vector<double> y_pos;
        std::vector<double> yaw_pos;
        for (size_t i = 0; i < path.size(); ++i)
        {
            auto* s = path[i]->as<ob::SE2StateSpace::StateType>();
            // std::cout << "Pose " << i << ": x=" << s->getX() << ", y=" << s->getY() << ", yaw=" << s->getYaw() << std::endl;
            x_pos.push_back(s->getX());
            y_pos.push_back(s->getY());
            yaw_pos.push_back(s->getYaw());
        }

        /* Free memory */
        for (auto* s : path) {
            space->freeState(s);
        }

        return {x_pos, y_pos, yaw_pos};
    };

    RenderModule::Init(980, 720, 1.0);
    RenderModule::EnableRootWindowDocking();
    RenderModule::RegisterImGuiCallback([&]() {
        // create radio-buttons for goal orientation
        ImGui::Begin("Goal Orientation");
        ImGui::SetWindowSize(ImVec2(200, 100), ImGuiCond_Once);
        ImGui::SetWindowPos(ImVec2(10, 720 - 100), ImGuiCond_Once);
        ImGui::Text("Goal Orientation:");
        static int yaw_deg;
        ImGui::RadioButton("0 degrees", &yaw_deg, 0.0);
        ImGui::RadioButton("90 degrees", &yaw_deg, 90.0);
        ImGui::RadioButton("180 degrees", &yaw_deg, 180.0);
        ImGui::RadioButton("270 degrees", &yaw_deg, 270.0);
        goal[2] = yaw_deg * M_PI / 180.0;  
        // if (ImGui::SliderFloat("Turning Radius", &turning_radius, 1.0f, 10.0f, "%.1f m")) {
        //     float step = 1.0f;
        //     turning_radius = std::round(turning_radius / step) * step;
        // }
        ImGui::SliderFloat("Turning Radius", &turning_radius, 1.0f, 10.0f, "%.1f m");
        // ImGui::DragFloat("Turning Radius", &turning_radius, 1.0f, 1.0f, 10.0f, "%.1f m");
        space->setTurningRadius(turning_radius);
        ImGui::End();
        
        ImGui::Begin("Line Plot");
        ImGui::SetWindowSize(ImVec2(860, 700), ImGuiCond_Once);
        ImGui::SetWindowPos(ImVec2(10, 10), ImGuiCond_Once);
        // auto mouse_pos = ImGui::GetMousePos();
        // std::cout << "Mouse position: x=" << mouse_pos.x << ", y=" << mouse_pos.y << std::endl;
        if (ImPlot::BeginPlot("Positions", ImVec2(-1, -1), ImPlotFlags_Equal)) {
            //set axis range
            // ImPlot::SetupAxes("X", "Y", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxes("X", "Y");
            ImPlot::SetupAxisLimits(ImAxis_X1, -10, 10, ImPlotCond_Once);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -10, 10, ImPlotCond_Once);
            auto mouse_pos = ImPlot::GetPlotMousePos();
            if (mouse_pos.x < 20 && mouse_pos.y < 20) {
                // plot orientation with marker and line
                std::vector<double> xs = {0.0, 1.0};
                std::vector<double> ys = {0.0, 0.0};
                ImPlot::PlotLine("start", xs.data(), ys.data(), xs.size());
                double marker_x = xs[0];
                double marker_y = ys[0];
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 6.0f);  // marker type and size
                ImPlot::PlotScatter("Marker", &marker_x, &marker_y, 1);

                xs = {mouse_pos.x, mouse_pos.x + std::cos(goal[2])};
                ys = {mouse_pos.y, mouse_pos.y + std::sin(goal[2])};
                ImPlot::PlotLine("goal", xs.data(), ys.data(), xs.size());
                marker_x = xs[0];
                marker_y = ys[0];
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 6.0f);  // marker type and size
                ImPlot::PlotScatter("Marker Goal", &marker_x, &marker_y, 1);

                data path_data = get_path(mouse_pos);
                ImPlot::PlotLine("Position", path_data.x_pos.data(), path_data.y_pos.data(), path_data.x_pos.size());

            }
            ImPlot::EndPlot();
        }
        ImGui::End();
    });
    RenderModule::Run();
    RenderModule::Shutdown(); 

    return 0;
}