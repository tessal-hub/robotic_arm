#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <WebServer.h>

class ArmController;
class WifiManager;
class JointModel;
class NvsStore;

/**
 * Web app điều khiển (WebServer đồng bộ, polling 250-300ms).
 * Tabs: Dashboard | Joints | Homing | WiFi. Cart/Draw sẽ thêm ở P2/P3.
 * Endpoint:
 *   GET  /                     trang chính (PROGMEM)
 *   GET  /api/status           JSON tổng hợp
 *   POST /api/jog              axis,deg        jog tương đối
 *   GET  /api/stop             dừng tất cả
 *   GET  /api/home/all         homing chuỗi J1..J4
 *   GET  /api/home/axis?axis=  homing 1 khớp (0-based, chỉ 0..3)
 *   GET  /api/sethome?axis=    đặt home tại chỗ (mọi khớp, kể cả J5/J6)
 *   GET  /api/clearcalib?axis= xoá calib NVS của khớp
 *   POST /api/wifi             ssid,pass       lưu NVS + restart
 */
void webBegin(WebServer& server, ArmController* arm, WifiManager* wifi,
              JointModel* joints, NvsStore* nvs);

#endif // WEB_SERVER_H
