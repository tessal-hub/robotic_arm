#include <cstdio>
#include <fstream>
#include <string>

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        std::printf("FAIL: %s (line %d)\n", msg, __LINE__); \
        ++g_fail; \
    } \
} while (0)

static std::string readSource(const char* path) {
    std::ifstream input(path);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

int main() {
    const std::string source = readSource("src/web_server.cpp");
    CHECK(!source.empty(), "web_server source available");
    CHECK(source.find("return requestCommand(url, {method:'POST'}, trigger);") != std::string::npos,
          "embedded UI sends command helper requests as POST");
    CHECK(source.find("function activateTab(tab, moveFocus = false)") != std::string::npos &&
          source.find("tab.addEventListener('keydown'") != std::string::npos,
          "navigation tabs implement keyboard activation");
    CHECK(source.find("class=\"preset-pill\"") != std::string::npos &&
          source.find("<span class=\"preset-pill\"") == std::string::npos,
          "Cartesian presets are semantic buttons, not clickable spans");
    CHECK(source.find("confirmSetHome(") != std::string::npos,
          "Set Home requests confirmation before replacing a calibration mark");
    CHECK(source.find("pointerdown") != std::string::npos &&
          source.find("isPaneActive('pane-dash')") != std::string::npos,
          "3D canvas supports pointer input and skips hidden dashboard rendering");
    CHECK(source.find("backdrop-filter") == std::string::npos &&
          source.find("transition: all") == std::string::npos,
          "UI avoids nonessential blur and broad transition properties");

    constexpr const char* mutationRoutes[] = {
        "/api/stop", "/api/home/all", "/api/home/axis", "/api/sethome", "/api/clearcalib", "/api/release/j1-j4"
    };
    for (const char* route : mutationRoutes) {
        const std::string post = std::string("server.on(\"") + route + "\", HTTP_POST";
        const std::string get = std::string("server.on(\"") + route + "\", HTTP_GET";
        CHECK(source.find(post) != std::string::npos, "state-changing route is POST");
        CHECK(source.find(get) == std::string::npos, "state-changing route is not GET");
    }

    CHECK(source.find("server.on(\"/api/draw/presets\", HTTP_GET, handleDrawProfiles);") != std::string::npos,
          "quick-draw profiles are a read-only endpoint");
    CHECK(source.find("server.on(\"/api/draw/preset\", HTTP_POST, handleDrawPreset);") != std::string::npos,
          "quick-draw command is a POST endpoint");
    CHECK(source.find("WORKPLANE_ENABLED") != std::string::npos,
          "quick-draw refuses base-Z profiles while WorkPlane UCS is enabled");
    CHECK(source.find("NOT_CALIBRATED") != std::string::npos,
          "workplane enable reports a missing calibration instead of silently staying off");
    CHECK(source.find("RELEASE_J1_J4") != std::string::npos &&
          source.find("confirmReleaseJ1J4") != std::string::npos,
          "J1-J4 torque release is queued and explicitly confirmed");
    CHECK(source.find("id=\"dwQuickShape\"") != std::string::npos &&
          source.find("Square") != std::string::npos &&
          source.find("Circle") != std::string::npos,
          "quick draw exposes Square and Circle shape choices");
    CHECK(source.find("id=\"dwQuickStartX\"") != std::string::npos &&
          source.find("id=\"dwQuickStartY\"") != std::string::npos &&
          source.find("id=\"dwQuickLength\"") != std::string::npos,
          "quick draw keeps editable start point controls");
    CHECK(source.find("id=\"dwA1\"") == std::string::npos &&
          source.find("id=\"dwA2\"") == std::string::npos &&
          source.find("id=\"dwA3\"") == std::string::npos &&
          source.find("id=\"dwA4\"") == std::string::npos,
          "manual start/end coordinate section is removed from drawing UI");
    CHECK(source.find("sx=${sx}&sy=${sy}${lineLength}") != std::string::npos &&
          source.find("makeSuggestedShape") != std::string::npos,
          "quick-draw custom shape start coordinates reach the safety validator");

    if (g_fail == 0) {
        std::printf("ALL PASSED (web API contract)\n");
        return 0;
    }
    std::printf("%d FAILED\n", g_fail);
    return 1;
}
