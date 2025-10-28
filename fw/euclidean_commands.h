#include <cmath>
#include <array>

struct EuclideanParams {
    float min_thrust;
    float beta_max;
    float side_length;
    float z_top;
    float half_side;
    float tan_beta;
};

inline EuclideanParams compute_prism_dims(float min_thrust, float beta_max) {
    EuclideanParams p{};
    p.min_thrust = min_thrust;
    p.beta_max = beta_max;

    float r_slice = min_thrust * std::tan(beta_max);
    p.side_length = std::sqrt(2.0f) * r_slice;
    p.half_side  = 0.5f * p.side_length;
    p.tan_beta   = std::tan(beta_max);

    float half = p.half_side;
    p.z_top = std::sqrt(std::max(0.0f, 1.0f - 2.0f * half * half));
    return p;
}

// Scale Euclidean command [-1,1]^3 to actuator polar coordinates
// Returns {T, beta, psi}
inline std::array<float, 3> scale_command(
    const EuclideanParams& p,
    float x_e, float y_e, float z_e)
{
    // Scale z: [-1,0] -> [z_top, 0]
    float fz = -z_e * p.z_top;

    // Scale x,y
    float fx = x_e * p.half_side;
    float fy = y_e * p.half_side;

    // Cone constraint
    float r2 = fx*fx + fy*fy;
    float r = std::sqrt(r2);
    float r_max = fz * p.tan_beta;
    if (r > r_max && r > 1e-6f) {
        float scale = r_max / r;
        fx *= scale;
        fy *= scale;
        r2 = fx*fx + fy*fy;
    }

    // Thrust magnitude
    float T2 = r2 + fz*fz;
    float T  = std::sqrt(T2);

    // Elevation and azimuth
    float beta = (T > 1e-6f) ? std::acos(fz / T) : 0.0f;
    float psi  = std::atan2(fy, fx);

    return {T, beta, psi};
}