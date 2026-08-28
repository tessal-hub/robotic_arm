#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <WebServer.h>

class ArmController;
class WifiManager;
class JointModel;
class NvsStore;
class WorkPlane;

/**
 * Web app điều khiển (WebServer đồng bộ, polling 250-300ms / High-Speed Telemetry).
 * Tabs: Dashboard | 3D Simulation | Joints | Homing | WiFi.
 * Endpoint:
 *   GET  /                     trang chính (PROGMEM)
 *   GET  /api/status           JSON tổng hợp
 *   POST /api/jog              axis,deg        jog tương đối
 *   GET  /api/stop             dừng tất cả
 *   POST /api/move             x,y,z,feed      chuyển động Cartesian
 *   POST /api/draw             shape,params    vẽ line/circle
 *   GET  /api/home/all         homing chuỗi J1..J4
 *   GET  /api/home/axis?axis=  homing 1 khớp (0-based, chỉ 0..3)
 *   GET  /api/sethome?axis=    đặt home tại chỗ (mọi khớp, kể cả J5/J6)
 *   GET  /api/clearcalib?axis= xoá calib NVS của khớp (Gated khi IDLE)
 *   POST /api/wifi             ssid,pass       lưu NVS + restart (Gated khi IDLE)
 *   POST /api/workplane/calib  p1,p2,p3        hiệu chuẩn mặt phẳng 3 điểm
 *   POST /api/workplane/toggle en=1/0          bật/tắt UCS
 *   GET  /api/workplane/status JSON trạng thái WorkPlane
 */
void webBegin(WebServer& server, ArmController* arm, WifiManager* wifi,
              JointModel* joints, NvsStore* nvs, WorkPlane* workPlane = nullptr);

#endif // WEB_SERVER_H
