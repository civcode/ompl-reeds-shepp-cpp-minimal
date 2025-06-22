#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <iostream>

namespace ob = ompl::base;

int main()
{
    auto space(std::make_shared<ob::ReedsSheppStateSpace>(1.0));
    /* bound are NOT required for Reeds-Shepp solutiong */
    // ob::RealVectorBounds bounds(2);
    // bounds.setLow(-10); bounds.setHigh(10);
    // space->setBounds(bounds);

    ob::ScopedState<> start(space);
    start[0] = 0.0; start[1] = 0.0; start[2] = 0.0;

    ob::ScopedState<> goal(space);
    goal[0] = 4.0; goal[1] = 3.0; goal[2] = M_PI / 2;

    std::cout << "Distance: " << space->distance(start.get(), goal.get()) << std::endl;
    return 0;
}