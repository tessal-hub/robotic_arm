#ifndef PLANNER_H
#define PLANNER_H

#include <Arduino.h>
#include "config.h"

class Motor;
class JointModel;

/**
 * Bộ thực thi chuyển động Cartesian phối hợp cho chế độ VẼ (bút hướng xuống).
 *
 * - Nguồn waypoint: bộ sinh theo hình (line/circle) — KHÔNG lưu buffer lớn,
 *   mỗi lần gọi sinh ra điểm kế tiếp cách nhau ~DRAW_SEGMENT_MM.
 * - Mỗi segment: IK pen-down -> góc khớp -> di chuyển ĐỒNG THỜI mọi trục với
 *   thời gian bằng nhau (trục nhiều step nhất chạy tốc độ feed, các trục khác
 *   được scale interval tương ứng) => đầu cuối segment chính xác, lệch trong
 *   segment cỡ sub-mm với DRAW_SEGMENT_MM = 1mm.
 * - Nâng/hạ bút: dịch Z thêm PEN_LIFT_MM bằng chính cơ chế segment.
 */
class Planner {
public:
    enum class Shape : uint8_t { NONE = 0, POINT, LINE, CIRCLE };

    struct Job {
        Shape shape{Shape::NONE};
        // POINT : tới (x1,y1,z) rồi nâng bút
        // LINE  : từ (x1,y1) tới (x2,y2)
        // CIRCLE: tâm (x1,y1), bán kính r
        float x1{0}, y1{0};
        float x2{0}, y2{0};
        float z{0};      // mặt giấy (Z vẽ)
        float r{0};
        float feedMmS{DRAW_FEED_MM_S};
        bool drawNow{true}; // true: hạ bút vẽ; false: chỉ di chuyển (travel)
    };

    enum class State : uint8_t {
        IDLE = 0,
        LIFTING,       // nâng bút khỏi giấy trước khi travel
        TRAVELING,     // di chuyển tới điểm bắt đầu (bút đang nâng)
        DROPPING,      // hạ bút xuống z vẽ
        DRAWING,       // đang sinh + chạy các segment
        FINISHED_LIFT  // nâng bút kết thúc
    };

    Planner();

    Planner(const Planner&) = delete;
    Planner& operator=(const Planner&) = delete;

    void begin(Motor** motors, JointModel* joints);

    [[nodiscard]] bool submit(const Job& job);
    void stop();                       // huỷ ngay, dừng motor
    [[nodiscard]] bool isActive() const noexcept { return state_ != State::IDLE; }
    [[nodiscard]] State state() const noexcept { return state_; }

    // Gọi định kỳ từ motion task (10ms). Sinh segment khi các trục đã dừng.
    void tick();

    [[nodiscard]] uint32_t segmentsDone() const noexcept { return segDone_; }

private:
    bool startMoveTo(float x, float y, float z, float feedMmS); // 1 segment tới đích
    bool nextDrawSegment();
    void finishAll();

    Motor* motors[NUM_MOTORS]{};
    JointModel* jm{nullptr};

    Job job_{};
    bool hasJob_{false};
    State state_{State::IDLE};

    // progress của bộ sinh
    float prog_{0.0f};          // LINE: quãng đường đã đi; CIRCLE: góc đã quét
    float totalLen_{0.0f};      // LINE tổng chiều dài; CIRCLE: chu vi phần vẽ
    float startAng_{0.0f};      // CIRCLE: góc bắt đầu
    float sweep_{0.0f};         // CIRCLE: cung quét (dương = CCW)
    float curX_{0}, curY_{0}, curZ_{0}; // vị trí Cartesian hiện tại (theo lệnh)
    uint32_t segDone_{0};
};

#endif // PLANNER_H
