from tempe_displays.st7789.pimoroni import PimoroniDisplay


# Change to match the characteristics of your display
SIZE = (320, 240)
CENTERED = False  # True for for round displays and the original Pico Display Pack
ROTATION = 0  # or 90, 180, 270


def init_display():
    display = PimoroniDisplay(size=SIZE, centered=CENTERED)
    await display.init(ROTATION)
    display.backlight_pin(1)
    return display
