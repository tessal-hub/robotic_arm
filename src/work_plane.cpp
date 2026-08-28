#include "work_plane.h"

bool WorkPlane::setThreePointCalibration(const Point3D& p1, const Point3D& p2, const Point3D& p3) {
    const Point3D v1 = p2 - p1;
    const Point3D v2 = p3 - p1;
    const float len1 = v1.length();
    const float len2 = v2.length();

    // 1. Distance validation (minimum 20mm baseline)
    if (len1 < 20.0f || len2 < 20.0f) {
        m_lastError = "LỖI: Các điểm hiệu chuẩn quá gần nhau (Khoảng cách tối thiểu 20mm)";
        m_isCalibrated = false;
        m_enabled = false;
        return false;
    }

    const Point3D normalVec = v1.cross(v2);
    const float normalLen = normalVec.length();
    const float sinAngle = normalLen / (len1 * len2);

    // 2. Collinear Degeneracy check: sin(phi) >= 0.1736 <=> phi >= 10 deg
    if (sinAngle < 0.1736f) {
        m_lastError = "LỖI SUY BIẾN: 3 điểm gần như thẳng hàng (Góc mở < 10°). Vui lòng chọn điểm P3 xa trục P1-P2.";
        m_isCalibrated = false;
        m_enabled = false;
        return false;
    }

    // 3. Orthonormal Basis Construction
    m_uAxis = v1 / len1;
    m_normal = normalVec / normalLen;
    m_vAxis = m_normal.cross(m_uAxis);
    m_origin = p1;

    m_isCalibrated = true;
    m_enabled = true;
    m_lastError = "";

    Serial.printf("[UCS] WorkPlane calibrated: Origin(%.1f, %.1f, %.1f), Normal(%.3f, %.3f, %.3f)\n",
                  m_origin.x, m_origin.y, m_origin.z, m_normal.x, m_normal.y, m_normal.z);
    return true;
}

Point3D WorkPlane::toRobotXYZ(float u, float v, float wLift) const {
    if (!m_enabled) {
        return {u, v, wLift}; // Fallback identity
    }
    return m_origin + (m_uAxis * u) + (m_vAxis * v) + (m_normal * wLift);
}

Point3D WorkPlane::fromRobotXYZ(const Point3D& robotP) const {
    if (!m_enabled) {
        return robotP;
    }
    const Point3D d = robotP - m_origin;
    return {
        d.dot(m_uAxis),
        d.dot(m_vAxis),
        d.dot(m_normal)
    };
}
