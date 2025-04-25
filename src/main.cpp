#include <cansocket/cansocket.hpp>

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/screen/string.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <print>

int main() {
    auto screen = ftxui::ScreenInteractive::FitComponent();
    ftxui::Component button = ftxui::Button({
        .label = "Click to quit",
        .on_click = screen.ExitLoopClosure(),
    });
    screen.Loop(button);
    std::println("Hello from cantty!");

    auto cansocket = std::make_unique<can::Socket>(std::cout);
    cansocket->connect("can0", "125000");

    return EXIT_SUCCESS;
}
