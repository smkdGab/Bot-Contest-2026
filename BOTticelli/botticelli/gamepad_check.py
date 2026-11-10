"""Offline input checklist, independent of the car dashboard and backends."""
import pygame


class Checklist:
    def __init__(self):
        self.buttons = []
        self.axes = []
        self.hats = []

    def update(self, buttons, axes, hats):
        if len(self.buttons) != len(buttons):
            self.buttons = [[False, False] for _ in buttons]
        if len(self.axes) != len(axes):
            self.axes = [[False, False] for _ in axes]
        if len(self.hats) != len(hats):
            self.hats = [set() for _ in hats]
        for history, pressed in zip(self.buttons, buttons):
            if pressed:
                history[0] = True
            elif history[0]:
                history[1] = True
        for history, value in zip(self.axes, axes):
            history[0] |= value < -0.7
            history[1] |= value > 0.7
        for history, (x, y) in zip(self.hats, hats):
            if x: history.add((x, 0))
            if y: history.add((0, y))


class CheckWindow:
    def __init__(self, config):
        pygame.display.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((960, 720))
        pygame.display.set_caption("BOTticelli / Gamepad checklist")
        self.font = pygame.font.Font(None, 24)
        self.title = pygame.font.Font(None, 36)
        self.config = config
        self.history = Checklist()
        self.generation = None

    def text(self, value, x, y, passed=False, title=False):
        color = (92, 218, 175) if passed else (219, 229, 237)
        self.screen.blit((self.title if title else self.font).render(value, True, color), (x, y))

    def draw(self, reader, sample):
        if self.generation != sample.generation:
            self.history = Checklist()
            self.generation = sample.generation
        self.screen.fill((16, 25, 33))
        self.text("Gamepad checklist", 30, 22, title=True)
        self.text(reader.message, 30, 65)
        self.text("Press and release A. Test the named driving controls and active D-pad directions.", 30, 100)
        if not sample.connected:
            self.text("Connect a controller to begin.", 30, 160)
        else:
            j = getattr(reader, "joystick", None)
            axes = ([j.get_axis(self.config.throttle_axis)*self.config.throttle_sign,
                     j.get_axis(self.config.steering_axis)*self.config.steering_sign] if j is not None
                    else [sample.throttle, sample.steering])
            triggers = self.config.throttle_mode == "triggers"
            if triggers and j is not None:
                axes[0] = (j.get_axis(self.config.forward_trigger_axis)-j.get_axis(self.config.reverse_trigger_axis))/2
            hats = [j.get_hat(self.config.hat)] if self.config.dpad and j is not None and j.get_numhats() > self.config.hat else []
            self.history.update([sample.a], axes, hats)
            done = all(self.history.buttons[0])
            self.text(f"{'[OK]' if done else '[  ]'} A - hold to drive, release to stop: "
                      f"{'PRESSED' if sample.a else 'released'}", 30, 170, done)
            throttle, steering = self.history.axes
            directions = [("RT - forward" if triggers else "Up - forward", throttle[1]),
                          ("LT - reverse" if triggers else "Down - reverse", throttle[0]),
                          ("Left - steer left", steering[1]), ("Right - steer right", steering[0])]
            self.text("Triggers / Left stick" if triggers else "Left stick", 30, 235, title=True)
            for row, (label, complete) in enumerate(directions):
                self.text(f"{'[OK]' if complete else '[  ]'} {label}", 30, 285+row*45, complete)
            if self.config.dpad:
                self.text("D-pad", 510, 235, title=True)
                seen = self.history.hats[0] if self.history.hats else set()
                for row, (label, direction) in enumerate(zip(
                        ("Up - forward", "Down - reverse", "Left - steer left", "Right - steer right"),
                        ((0, 1), (0, -1), (-1, 0), (1, 0)))):
                    self.text(f"{'[OK]' if direction in seen else '[  ]'} {label}", 510, 285+row*45, direction in seen)
            self.text(f"Throttle {sample.throttle:+.2f}   Steering {sample.steering:+.2f}", 30, 520)
            self.text("Completed checks stay green until the controller reconnects.", 30, 570)
        self.text("Driving: hold A to move, release A to stop. Stick may be moved before A. Close to exit.", 30, 685)
        pygame.display.flip()

    def close(self):
        pygame.quit()
