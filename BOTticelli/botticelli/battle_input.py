"""Keyboard/gamepad selection for the simulation-only battle window."""
from dataclasses import replace
import time

from .control import Input


class BattleInput:
    def __init__(self, gamepad):
        import pygame
        self.pg = pygame
        self.gamepad = gamepad
        self.pressed = set()
        self.focused = True
        self.source = None
        self.generation = 0
        self.pad_generation = None
        self.pad_ready = False
        self.previous_a = False
        self.message = 'Keyboard: WASD / arrows'
        self.movement = {pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d,
                         pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT}

    def poll(self, events):
        pg = self.pg
        pad = self.gamepad.poll(events)
        changed = self.pad_generation is not None and self.pad_generation != pad.generation
        if changed:
            self.pad_ready = False
            if self.source == 'gamepad':
                self.generation += 1
        self.pad_generation = pad.generation
        if not pad.connected:
            self.pad_ready = False
        elif not pad.a:
            self.pad_ready = True
        if self.source is None:
            self.source = 'gamepad' if pad.connected else 'keyboard'
        keyboard_requested = False
        focus_interrupted = False
        for event in events:
            if event.type == pg.WINDOWFOCUSLOST:
                self.focused = False
                focus_interrupted = True
                self.pressed.clear()
            elif event.type == pg.WINDOWFOCUSGAINED:
                self.focused = True
            elif event.type == pg.KEYUP:
                self.pressed.discard(event.key)
            elif event.type == pg.KEYDOWN and self.focused and not getattr(event, 'repeat', False):
                if event.key in self.movement:
                    self.pressed.add(event.key)
                    keyboard_requested = True
                elif event.key == pg.K_RETURN and not pad.connected:
                    # Enter deliberately rearms keyboard play after pad loss.
                    self.source = 'keyboard'
        if keyboard_requested:
            self.source = 'keyboard'
        elif self.focused and not self.pressed and pad.connected and self.pad_ready and pad.a and not self.previous_a:
            self.source = 'gamepad'
        self.previous_a = pad.a
        if not self.focused or focus_interrupted:
            self.message = 'Focus changed: Enter to resume'
            return Input(time.monotonic(), generation=self.generation)
        if self.source == 'gamepad':
            self.message = self.gamepad.message
            return replace(pad, generation=self.generation)
        self.message = 'Keyboard: WASD / arrows'
        down = self.pressed
        throttle = int(bool(down & {pg.K_w, pg.K_UP}))-int(bool(down & {pg.K_s, pg.K_DOWN}))
        steering = int(bool(down & {pg.K_a, pg.K_LEFT}))-int(bool(down & {pg.K_d, pg.K_RIGHT}))
        return Input(time.monotonic(), True, throttle, steering,
                     a=bool(down), generation=self.generation)
