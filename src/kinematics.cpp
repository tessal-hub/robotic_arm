#include "kinematics.h"

#include <cmath>
#include <initializer_list>

namespace kin {
namespace {

struct Mat4 {
    float m[4][4];

    static Mat4 identity() {
        Mat4 r{};
        r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0f;
        return r;
    }
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += a.m[i][k] * b.m[k][j];
            r.m[i][j] = s;
        }
    return r;
}

inline Mat4 rx(float deg) {
    const float c = cosf(deg * 0.017453292519943295f);
    const float s = sinf(deg * 0.017453292519943295f);
    Mat4 r = Mat4::identity();
    r.m[1][1] = c; r.m[1][2] = -s;
    r.m[2][1] = s; r.m[2][2] = c;
    return r;
}

inline Mat4 rz(float deg) {
    const float c = cosf(deg * 0.017453292519943295f);
    const float s = sinf(deg * 0.017453292519943295f);
    Mat4 r = Mat4::identity();
    r.m[0][0] = c; r.m[0][1] = -s;
    r.m[1][0] = s; r.m[1][1] = c;
    return r;
}

inline Mat4 tx(float d) { Mat4 r = Mat4::identity(); r.m[0][3] = d; return r; }
inline Mat4 tz(float d) { Mat4 r = Mat4::identity(); r.m[2][3] = d; return r; }

// Modified DH (Craig): T_i = Rx(a_{i-1}) · Tx(a_{i-1}) · Rz(theta_i) · Tz(d_i)
// Bảng: {a_{i-1}, alpha_{i-1}, d_i} — docs/ARM_GEOMETRY.md mục 3
struct DhRow {
    float a, alpha, d;
};

constexpr float TH_OFFSETS[6] = {0.0f, -90.0f, 0.0f, 0.0f, 0.0f, 0.0f};

} // namespace

FkResult forward(const float enc[6]) {
    // constexpr local (C++11-compatible init)
    static const DhRow DH[6] = {
        {0.0f, 0.0f, 139.0f}, {0.0f, -90.0f, 0.0f}, {138.0f, 0.0f, 0.0f},
        {88.0f, -90.0f, 126.0f}, {0.0f, 90.0f, 0.0f}, {0.0f, -90.0f, 0.0f},
    };

    Mat4 T = Mat4::identity();
    Mat4 T4 = T; // sẽ ghi đè sau vòng 4
    for (int i = 0; i < 6; ++i) {
        const float th = enc[i] + TH_OFFSETS[i];
        T = T * rx(DH[i].alpha) * tx(DH[i].a) * rz(th) * tz(DH[i].d);
        if (i == 3) T4 = T;
    }
    T = T * tz(D_TOOL); // bút gắn đồng trục J6, dài D_TOOL dọc trục tool

    FkResult out;
    out.wristCenter.x = T4.m[0][3];
    out.wristCenter.y = T4.m[1][3];
    out.wristCenter.z = T4.m[2][3];
    out.tcp.x = T.m[0][3];
    out.tcp.y = T.m[1][3];
    out.tcp.z = T.m[2][3];
    return out;
}

bool ikPenDown(const Pose& target, float outEnc[6]) {
    // Wrist center: bút chỉ xuống => tâm cổ tay cao hơn TCP đúng D_TOOL
    const float cx = target.x;
    const float cy = target.y;
    const float cz = target.z + D_TOOL;

    const float t1 = atan2f(cy, cx);

    const float r = sqrtf(cx * cx + cy * cy);
    const float h = cz - D1;
    const float dist = sqrtf(r * r + h * h);

    // Kiểm tra vùng với của cơ cấu phẳng 2 khâu (A2 và L_FORE)
    if (dist > (A2 + L_FORE) || dist < fabsf(A2 - L_FORE)) return false;

    // Law of cosines tại khớp vai
    float cb = (A2 * A2 + dist * dist - L_FORE * L_FORE) / (2.0f * A2 * dist);
    if (cb > 1.0f) cb = 1.0f;
    if (cb < -1.0f) cb = -1.0f;
    const float beta = acosf(cb);
    const float gamma = atan2f(h, r);
    const float deltaRad = DELTA_WRIST * 0.017453292519943295f; // deg -> rad

    bool found = false;
    float bestEnc[6] = {0, 0, 0, 0, 0, 0};
    float bestJ3 = 1e9f;

    for (const int sign : {+1, -1}) {
        const float phi2 = gamma + sign * beta; // độ cao khâu trên
        const float t2 = -phi2;

        const float rRem = r - A2 * cosf(phi2);
        const float hRem = h - A2 * sinf(phi2);
        const float phiPsi = atan2f(hRem, rRem);   // độ cao khâu trước hiệu dụng

        // q23 = t2_DH + t3_DH ; quan hệ: hướng fore-arm = -(q23 + DELTA)
        const float q23 = -phiPsi - deltaRad;
        const float t3 = q23 - t2;

        const float e1 = t1 * 57.29577951308232f;
        const float e2 = t2 * 57.29577951308232f + 90.0f; // trừ offset -90
        const float e3 = t3 * 57.29577951308232f;
        const float e5 = -q23 * 57.29577951308232f;       // t5_DH = -q23

        if (!(e2 >= J2_MIN && e2 <= J2_MAX)) continue;
        if (!(e3 >= J3_MIN && e3 <= J3_MAX)) continue;
        if (!(e5 >= J5_MIN && e5 <= J5_MAX)) continue;

        if (!found || fabsf(e3) < bestJ3) {
            bestEnc[0] = e1;
            bestEnc[1] = e2;
            bestEnc[2] = e3;
            bestEnc[3] = 0.0f;                       // t4 giữ mặt phẳng tay
            bestEnc[4] = e5;
            bestEnc[5] = 0.0f;                       // roll khoá 0 khi vẽ
            bestJ3 = fabsf(e3);
            found = true;
        }
    }

    if (!found) return false;
    for (int i = 0; i < 6; ++i) outEnc[i] = bestEnc[i];
    return true;
}

} // namespace kin
