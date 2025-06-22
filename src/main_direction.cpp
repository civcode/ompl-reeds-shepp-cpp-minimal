#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <iostream>

namespace ob = ompl::base;

class MyReedsShepp : public ob::ReedsSheppStateSpace {
public:
    using ob::ReedsSheppStateSpace::ReedsSheppStateSpace;
    /* Wrapper for interpolate using ob::ReedsSheppStateSpace::ReedsSheppPath */
    void interpolate(const ob::State *from, const ob::ReedsSheppStateSpace::ReedsSheppPath &path, double t, ob::State *state) const {
        ReedsSheppStateSpace::interpolate(from, path, t, state);
    }
    /* Wrapper for setTurningRadius using ob::ReedsSheppStateSpace::setTurningRadius */
    void setTurningRadius(double rho) {
        rho_ = rho;
    }
};

int main() {
    auto space = std::make_shared<MyReedsShepp>(1.0);
    /* bound are NOT required for Reeds-Shepp solutiong */
    // auto space = std::make_shared<ob::ReedsSheppStateSpace>(1.0);
    // ob::RealVectorBounds bounds(2);
    // bounds.setLow(-10); bounds.setHigh(10);
    // space->setBounds(bounds);

    ob::ScopedState<> start(space);
    start[0] = 0.0; start[1] = 0.0; start[2] = 0.0;

    ob::ScopedState<> goal(space);
    goal[0] = 8.0; goal[1] = 7.0; goal[2] = M_PI;

    /* Compute Reeds-Shepp path */
    space->setTurningRadius(4.0); 
    auto path = space->reedsShepp(start.get(), goal.get());

    /* Interpolate path poses using */
    std::vector<ob::State*> path_states;
    int path_length = 50;
    for (int i = 0; i <= path_length; ++i) {
        double t = static_cast<double>(i) / path_length;
        ob::State *s = space->allocState();
        space->interpolate(start.get(), path, t, s);
        path_states.push_back(s);
    }
    std::cout << "path.size() = " << path_states.size() << std::endl;

    /* Get path segments and driving direction */
    double distance = 0;
    for (int i = 0; i < 5; ++i) {
        auto length = path.length_[i];
        distance += std::abs(length);
        if (std::abs(length) < 1e-6) continue;

        std::string motionType;
        switch (path.type_[i]) {
            // case ob::ReedsSheppStateSpace::RS_LEFT:     motionType = "Left"; break;
            // case ob::ReedsSheppStateSpace::RS_RIGHT:    motionType = "Right"; break;
            // case ob::ReedsSheppStateSpace::RS_STRAIGHT: motionType = "Straight"; break;
            case ob::ReedsSheppStateSpace::RS_LEFT:     motionType = "L"; break;
            case ob::ReedsSheppStateSpace::RS_RIGHT:    motionType = "R"; break;
            case ob::ReedsSheppStateSpace::RS_STRAIGHT: motionType = "S"; break;
            default: motionType = "None"; break;
        }

        std::string direction = (length > 0) ? "Forward" : "Reverse";
        std::cout << "Segment " << i << ": " << motionType << ", " << direction << ", length = " << length << std::endl;
    }
    std::cout << "Total distance: " << distance << std::endl;

    return 0;
}