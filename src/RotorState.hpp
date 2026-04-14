#ifndef ROTOR_STATE_HPP
#define ROTOR_STATE_HPP

#include "Math.hpp"

namespace yasim {

struct State {
    double pos[3];
    float orient[9];
    float v[3];
    float rot[3];
    float acc[3];
    float racc[3];

    State() {
        int i;
        for(i=0; i<3; i++) {
            pos[i] = v[i] = rot[i] = acc[i] = racc[i] = 0;
            int j;
            for(j=0; j<3; j++) {
                orient[3*i + j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    void posLocalToGlobal(const float* lpos, double* gpos) const {
        float tmp[3];
        Math::tmul33(orient, lpos, tmp);
        gpos[0] = tmp[0] + pos[0];
        gpos[1] = tmp[1] + pos[1];
        gpos[2] = tmp[2] + pos[2];
    }

    void posGlobalToLocal(const double* gpos, float* lpos) const {
        lpos[0] = (float)(gpos[0] - pos[0]);
        lpos[1] = (float)(gpos[1] - pos[1]);
        lpos[2] = (float)(gpos[2] - pos[2]);
        Math::vmul33(orient, lpos, lpos);
    }

    void velLocalToGlobal(const float* lvel, float* gvel) const {
        Math::tmul33(orient, lvel, gvel);
    }

    void velGlobalToLocal(const float* gvel, float* lvel) const {
        Math::vmul33(orient, gvel, lvel);
    }

    void planeGlobalToLocal(const double* gplane, float* lplane) const {
        lplane[0] = (float)-gplane[0];
        lplane[1] = (float)-gplane[1];
        lplane[2] = (float)-gplane[2];
        Math::vmul33(orient, lplane, lplane);
        lplane[3] = (float)(pos[0] * gplane[0] + pos[1] * gplane[1]
            + pos[2] * gplane[2] - gplane[3]);
    }
};

} // namespace yasim

#endif
