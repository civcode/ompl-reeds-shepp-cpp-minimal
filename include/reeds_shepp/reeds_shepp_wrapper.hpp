#ifndef REEDS_SHEPP_WRAPPER_HPP_
#define REEDS_SHEPP_WRAPPER_HPP_

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;

class ReedsSheppWrapper : public ob::ReedsSheppStateSpace {
public:
    using ob::ReedsSheppStateSpace::ReedsSheppStateSpace;
    /* Wrapper for interpolate */
    void interpolate(const ob::State *from, const ob::State *to, double t, ob::State *state) const {
        ReedsSheppStateSpace::interpolate(from, to, t, state);
    }
    /* Wrapper for interpolate using ob::ReedsSheppStateSpace::ReedsSheppPath */
    void interpolate(const ob::State *from, const ob::ReedsSheppStateSpace::ReedsSheppPath &path, double t, ob::State *state) const {
        ReedsSheppStateSpace::interpolate(from, path, t, state);
    }
    /* Wrapper for setTurningRadius using ob::ReedsSheppStateSpace::setTurningRadius */
    void setTurningRadius(double rho) {
        rho_ = rho;
    }
    double getTurningRadius() const {
        return rho_;
    }
};

#endif // REEDS_SHEPP_WRAPPER_HPP_