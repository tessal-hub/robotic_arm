#pragma once
#include <Arduino.h>
#include <cmath>

struct Point3D {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    Point3D() = default;
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Point3D operator+(const Point3D& b) const { return {x + b.x, y + b.y, z + b.z}; }
    Point3D operator-(const Point3D& b) const { return {x - b.x, y - b.y, z - b.z}; }
    Point3D operator*(float s) const { return {x * s, y * s, z * s}; }
    Point3D operator/(float s) const { return {x / s, y / s, z / s}; }

    [[nodiscard]] float length() const { return std::sqrt(x * x + y * y + z * z); }
    [[nodiscard]] Point3D normalized() const {
        const float l = length();
        return (l > 1e-6f) ? (*this / l) : Point3D(0, 0, 0);
    }
    [[nodiscard]] Point3D cross(const Point3D& b) const {
        return {
            y * b.z - z * b.y,
            z * b.x - x * b.z,
            x * b.y - y * b.x
        };
    }
    [[nodiscard]] float dot(const Point3D& b) const {
        return x * b.x + y * b.y + z * b.z;
    }
};

/**
 * @brief User Coordinate System (UCS) / Work Plane for Arbitrary Drawing Surface
 * 
 * Supports 3-Point Calibration:
 * - P1: Origin (u=0, v=0)
 * - P2: Primary X-axis guide (+u)
 * - P3: Plane orientation guide point (+v)
 * 
 * Guards against collinear degeneracy (sin(phi) >= 0.1736 <=> phi >= 10 deg, dist >= 20mm).
 */
class WorkPlane {
private:
    Point3D m_origin{0.0f, 0.0f, 0.0f};
    Point3D m_uAxis{1.0f, 0.0f, 0.0f};
    Point3D m_vAxis{0.0f, 1.0f, 0.0f};
    Point3D m_normal{0.0f, 0.0f, 1.0f};
    bool    m_isCalibrated{false};
    bool    m_enabled{false};
    String  m_lastError{""};

public:
    WorkPlane() = default;

    /**
     * @brief Calibrate plane from 3 physical touch points
     * @return true if valid plane, false if degenerate / collinear
     */
    bool setThreePointCalibration(const Point3D& p1, const Point3D& p2, const Point3D& p3);

    /**
     * @brief Transform 2D plane coordinate (u, v, w_lift) to 3D Robot Base Coordinate (x, y, z)
     */
    [[nodiscard]] Point3D toRobotXYZ(float u, float v, float wLift = 0.0f) const;

    /**
     * @brief Transform 3D Robot Base Coordinate (x, y, z) to 2D plane coordinate (u, v, w)
     */
    [[nodiscard]] Point3D fromRobotXYZ(const Point3D& robotP) const;

    void setEnabled(bool en) noexcept { m_enabled = en && m_isCalibrated; }
    [[nodiscard]] bool isEnabled() const noexcept { return m_enabled; }
    [[nodiscard]] bool isCalibrated() const noexcept { return m_isCalibrated; }
    [[nodiscard]] const String& getLastError() const noexcept { return m_lastError; }
    [[nodiscard]] Point3D getOrigin() const noexcept { return m_origin; }
    [[nodiscard]] Point3D getNormal() const noexcept { return m_normal; }

    void resetToDefault() {
        m_origin = Point3D(0.0f, 0.0f, 0.0f);
        m_uAxis = Point3D(1.0f, 0.0f, 0.0f);
        m_vAxis = Point3D(0.0f, 1.0f, 0.0f);
        m_normal = Point3D(0.0f, 0.0f, 1.0f);
        m_isCalibrated = false;
        m_enabled = false;
        m_lastError = "";
    }
};
