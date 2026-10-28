#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <WebServer.h>

class ArmController;
class WifiManager;
class JointModel;
class NvsStore;

/**
 * Web app điều khiển (WebServer đồng bộ, polling 250-300ms / High-Speed Telemetry).
 * Tabs: Dashboard | 3D Simulation & Teach Studio | Joints | Homing | WiFi.
 * Endpoint:
 *   GET  /                     trang chính (PROGMEM)
 *   GET  /api/status           JSON tổng hợp
 *   POST /api/jog              axis,deg        jog tương đối
 *   GET  /api/stop             dừng tất cả
 *   POST /api/showoff                          trình diễn đồng bộ 6 khớp (Show Off dance)
 *   GET  /api/home/all         homing chuỗi J1..J4
 *   GET  /api/home/axis?axis=  homing 1 khớp (0-based, chỉ 0..3)
 *   GET  /api/sethome?axis=    đặt home tại chỗ (mọi khớp, kể cả J5/J6)
 *   GET  /api/clearcalib?axis= xoá calib NVS của khớp (Gated khi IDLE)
 *   POST /api/wifi             ssid,pass       lưu NVS + restart (Gated khi IDLE)
 *   POST /api/teach/free       free=1/0        thả lỏng J1..J3 kéo tay / khóa lại
 *   POST /api/teach/record     pen=1/0         ghi điểm vẽ (hạ bút) / nhấc bút
 *   POST /api/teach/undo                       hoàn tác điểm cuối
 *   POST /api/teach/clear                      xóa toàn bộ điểm
 *   GET  /api/teach/points                     danh sách điểm JSON
 *   POST /api/teach/load       body: JSON      nạp danh sách điểm từ JSON
 *   POST /api/teach/play       step,delay      chạy lại quỹ đạo dạy
 *   POST /api/teach/stop                       dừng chạy lại
 *   POST /api/teach/nvs/save                   lưu quỹ đạo vào Flash (NVS)
 *   POST /api/teach/nvs/load                   nạp quỹ đạo từ Flash (NVS)
 */
void webBegin(WebServer& server, ArmController* arm, WifiManager* wifi,
              JointModel* joints, NvsStore* nvs);

#endif // WEB_SERVER_H
