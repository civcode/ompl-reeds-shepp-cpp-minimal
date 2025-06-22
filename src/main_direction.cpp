#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <iostream>

namespace ob = ompl::base;

class MyReedsShepp : public ob::ReedsSheppStateSpace {
public:
    using ob::ReedsSheppStateSpace::ReedsSheppStateSpace;
    ReedsSheppPath getRawPath(const ob::State *s1, const ob::State *s2) const {
        return reedsShepp(s1, s2);
    }
};

int main() {
    auto space = std::make_shared<MyReedsShepp>(1.0);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10); bounds.setHigh(10);
    space->setBounds(bounds);

    ob::ScopedState<> start(space);
    start[0] = 0.0; start[1] = 0.0; start[2] = 0.0;

    ob::ScopedState<> goal(space);
    // goal[0] = 5.0; goal[1] = 5.0; goal[2] = M_PI / 2;
    goal[0] = 0.0; goal[1] = 3.0; goal[2] = M_PI/2;

    auto path = space->getRawPath(start.get(), goal.get());

    for (int i = 0; i < 5; ++i) {
        auto length = path.length_[i];
        if (std::abs(length) < 1e-6) continue;

        std::string motionType;
        switch (path.type_[i]) {
            case ob::ReedsSheppStateSpace::RS_LEFT:     motionType = "Left"; break;
            case ob::ReedsSheppStateSpace::RS_RIGHT:    motionType = "Right"; break;
            case ob::ReedsSheppStateSpace::RS_STRAIGHT: motionType = "Straight"; break;
            default: motionType = "None"; break;
        }

        std::string direction = (length > 0) ? "Forward" : "Reverse";
        std::cout << "Segment " << i << ": " << motionType << ", " << direction << ", length = " << length << std::endl;
    }

    return 0;
}