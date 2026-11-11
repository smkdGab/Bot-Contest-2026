"""The only window: pygame input, arena and shared LiDAR heatmap."""
import math
import time

import pygame

from .control import Input
from .display import range_color, distance, closest


class GamepadReader:
    def __init__(self, config):
        self.config = config
        self.joystick = None
        self.generation = 0
        self.message = "No controller"
        pygame.joystick.init()

    def poll(self, events):
        c = self.config
        if self.joystick is not None and any(e.type == pygame.JOYDEVICEREMOVED
                and e.instance_id == self.joystick.get_instance_id() for e in events):
            self.joystick.quit()
            self.joystick = None
        try:
            if self.joystick is None and pygame.joystick.get_count() > c.index:
                self.joystick = pygame.joystick.Joystick(c.index)
                self.joystick.init()
                self.generation += 1
            j = self.joystick
            if j is None:
                return Input(time.monotonic(), generation=self.generation)
            if c.a_button >= j.get_numbuttons():
                raise ValueError("Configured button missing; use gamepad-check")
            required = [c.steering_axis, c.throttle_axis] if c.throttle_mode == "stick" else [c.steering_axis, c.forward_trigger_axis, c.reverse_trigger_axis]
            if max(required) >= j.get_numaxes():
                raise ValueError("Configured axis missing; use gamepad-check")
            throttle = (j.get_axis(c.throttle_axis)*c.throttle_sign if c.throttle_mode == "stick" else
                        (j.get_axis(c.forward_trigger_axis)-j.get_axis(c.reverse_trigger_axis))/2)
            steering = j.get_axis(c.steering_axis)*c.steering_sign
            if c.dpad and j.get_numhats():
                if c.hat >= j.get_numhats():
                    raise ValueError("Configured D-pad hat missing")
                x, y = j.get_hat(c.hat)
                if y: throttle = y
                if x: steering = -x
            self.message = j.get_name()
            return Input(time.monotonic(), True, throttle, steering,
                         bool(j.get_button(c.lb_button)) if c.lb_button < j.get_numbuttons() else False,
                         bool(j.get_button(c.a_button)),
                         bool(j.get_button(c.b_button)) if c.b_button < j.get_numbuttons() else False,
                         self.generation)
        except (pygame.error, ValueError) as error:
            self.message = str(error)
            return Input(time.monotonic(), generation=self.generation)


class Dashboard:
    LIDAR_GRID = pygame.Rect(96, 785, 768, 80)
    BG = (9, 13, 19)
    PANEL = (17, 23, 32)
    LINE = (35, 44, 57)
    INK = (231, 238, 245)
    MUTED = (111, 129, 148)
    TEAL = (80, 235, 184)
    BLUE = (95, 167, 255)
    AMBER = (255, 186, 103)

    def __init__(self, config, title="BOTticelli"):
        from collections import deque
        from .simulation import RayCaster
        pygame.display.init()
        pygame.font.init()
        self.window = pygame.display.set_mode((1440, 960), pygame.RESIZABLE)
        self.screen = pygame.Surface((1440, 960))
        pygame.display.set_caption(title)
        face = pygame.font.match_font("inter,dejavusans")
        self.font = pygame.font.Font(face, 18)
        self.small = pygame.font.Font(face, 13)
        self.large = pygame.font.Font(face, 28)
        self.metric = pygame.font.Font(face, 46)
        self.config = config
        self.scale = min(472/config.arena.width, 472/config.arena.height)
        self.rays = RayCaster(config.lidar, []).rays
        self.layer = pygame.Surface((1440, 960), pygame.SRCALPHA)
        self.heatmap = pygame.Surface((768, 80))
        self.heatmap.fill((27, 35, 46))
        self.heatmap_key = None
        self.nearest = None
        self.closest_cell = None
        self.selected = None
        self.vehicle_cache = {}
        self.imu_history = deque(maxlen=100)
        self.imu_key = None

    def text(self, text, x, y, color=None, font=None):
        self.screen.blit((font or self.font).render(str(text), True, color or self.INK), (round(x), round(y)))

    def panel(self, rect, title, tag=None):
        x, y, w, h = rect
        pygame.draw.rect(self.screen, (4, 8, 13), (x, y+5, w, h), border_radius=18)
        pygame.draw.rect(self.screen, self.PANEL, rect, border_radius=18)
        pygame.draw.rect(self.screen, self.LINE, rect, 1, border_radius=18)
        self.text(title, x+22, y+18, self.MUTED, self.small)
        if tag:
            tint = self.AMBER if tag in ("STALE", "UNAVAILABLE") else self.MUTED if tag == "OFFLINE" else self.TEAL
            self.text(tag, x+w-90, y+18, tint, self.small)

    def point(self, x, y):
        return round(60+x*self.scale), round(674-y*self.scale)

    def rect_body(self, x, y, length, width, yaw, color):
        co, si = math.cos(yaw), math.sin(yaw)
        points = [self.point(x+co*dx-si*dy, y+si*dx+co*dy) for dx, dy in
                  [(-length/2, -width/2), (length/2, -width/2), (length/2, width/2), (-length/2, width/2)]]
        pygame.draw.polygon(self.screen, color, points)
        pygame.draw.aalines(self.screen, tuple(min(255,v+25) for v in color), True, points)

    def vehicle(self, color, steering):
        key = (color, round(math.degrees(steering)/2)*2)
        if key in self.vehicle_cache:
            return self.vehicle_cache[key]
        body = pygame.Surface((260, 172), pygame.SRCALPHA)
        pygame.draw.ellipse(body, (0, 0, 0, 90), (22, 25, 224, 142))
        for front in (-1, 1):
            for side in (-1, 1):
                tyre = pygame.Surface((62, 30), pygame.SRCALPHA)
                pygame.draw.rect(tyre, (73, 84, 99), (1, 1, 60, 28), border_radius=8)
                pygame.draw.rect(tyre, (8, 12, 18), (4, 4, 54, 22), border_radius=6)
                for t in (12, 23, 34, 45):
                    pygame.draw.line(tyre, (38, 47, 60), (t, 6), (t, 23), 2)
                tyre = pygame.transform.rotozoom(tyre, key[1] if front == 1 else 0, 1)
                body.blit(tyre, tyre.get_rect(center=(130+front*69, 86+side*56)))
        pygame.draw.rect(body, (7, 12, 20), (24, 37, 211, 100), border_radius=26)
        pygame.draw.rect(body, color, (28, 39, 203, 92), border_radius=25)
        pygame.draw.rect(body, tuple(round(v*0.7) for v in color), (34, 45, 190, 78), 2, border_radius=20)
        pygame.draw.rect(body, (25, 42, 53), (87, 48, 58, 74), border_radius=9)
        pygame.draw.rect(body, (100, 149, 164), (130, 51, 10, 68), border_radius=3)
        pygame.draw.rect(body, (213, 255, 245), (213, 50, 7, 20), border_radius=3)
        pygame.draw.rect(body, (213, 255, 245), (213, 100, 7, 20), border_radius=3)
        for yy in (51, 108):
            pygame.draw.rect(body, (237, 91, 105), (32, yy, 5, 12), border_radius=2)
        pygame.draw.circle(body, (14, 24, 33), (166, 86), 13)
        pygame.draw.circle(body, (205, 249, 244), (166, 86), 6)
        pygame.draw.line(body, (220, 255, 242), (175, 86), (203, 86), 2)
        self.vehicle_cache[key] = body
        return body

    def car(self, pose, length, width, color, steering=0, label=None):
        image = pygame.transform.rotozoom(self.vehicle(color, steering), math.degrees(pose[2]), length*self.scale/203)
        center = self.point(*pose[:2])
        self.screen.blit(image, image.get_rect(center=center))
        if label:
            self.text(label, center[0]+20, center[1]-27, color, self.small)

    def status(self, reading, now, limit=0.5):
        if reading is None: return "OFFLINE", self.MUTED
        if "error" in reading.values: return "UNAVAILABLE", self.AMBER
        return ("LIVE", self.TEAL) if 0 <= now-reading.received_at <= limit else ("STALE", self.AMBER)

    def viewport(self):
        width, height = self.window.get_size()
        ratio = min(width/1440, height/960)
        return pygame.Rect(0, 0, max(1, round(1440*ratio)), max(1, round(960*ratio))).move(
            (width-max(1, round(1440*ratio)))//2, (height-max(1, round(960*ratio)))//2)

    def lidar_cell(self, position):
        """Convert actual window pixels through letterboxing to wire row/column."""
        view = self.viewport()
        if not view.collidepoint(position):
            return None
        x, y = (position[0]-view.x)*1440/view.width, (position[1]-view.y)*960/view.height
        grid = self.LIDAR_GRID
        if grid.left <= x < grid.right and grid.top <= y < grid.bottom:
            return int((y-grid.y)*8/grid.height), int((x-grid.x)*64/grid.width)
        return None

    def draw(self, observation, command, state, now=None, source="SIM", controller=""):
        now = time.monotonic() if now is None else now
        a, c, l = self.config.arena, self.config.car, self.config.lidar
        sensors = {r.name:r for r in observation.sensors}
        self.screen.fill(self.BG)
        pygame.draw.rect(self.screen, self.TEAL, (25, 26, 26, 26), border_radius=7)
        pygame.draw.line(self.screen, self.BG, (32, 40), (44, 32), 3)
        self.text("BOTticelli", 66, 20, font=self.large)
        self.text("/  DRIVER STUDIO", 230, 34, self.MUTED, self.small)
        self.text(source, 1290, 32, self.TEAL)
        pygame.draw.line(self.screen, self.LINE, (24, 70), (1416, 70))
        pygame.draw.rect(self.screen, self.PANEL, (24, 88, 1392, 56), border_radius=12)
        active = "Driving" in state
        status = "CONTACT" if observation.collision else ("DRIVING" if active and command.throttle else "ARMED" if active else "STANDBY")
        if "disconnected" in state.lower() or "stale" in state.lower(): status = "DISCONNECTED"
        if "failed" in state.lower(): status = "FAULT"
        color = self.TEAL if active else self.AMBER if status in ("CONTACT", "FAULT", "DISCONNECTED") else self.MUTED
        pygame.draw.circle(self.screen, color, (49, 116), 4)
        self.text(status, 65, 103, color)
        self.text(controller[:43], 330, 107, self.MUTED, self.small)
        self.text(f"THROTTLE   {command.throttle:+.2f}", 823, 106, font=self.small)
        self.text(f"STEERING   {math.degrees(command.steering*math.radians(c.max_steering_deg)):+.0f}°", 1103, 106, font=self.small)
        self.panel((24, 164, 912, 544), "ARENA", f"{a.width:g} × {a.height:g} M")
        world = pygame.Rect(*self.point(0, a.height), round(a.width*self.scale), round(a.height*self.scale))
        pygame.draw.rect(self.screen, (13, 23, 31), world)
        for ix in range(round(a.width*4)+1):
            pygame.draw.line(self.screen, (25, 37, 48) if ix%4==0 else (18, 29, 39), self.point(ix/4,0), self.point(ix/4,a.height))
        for iy in range(round(a.height*4)+1):
            pygame.draw.line(self.screen, (25, 37, 48) if iy%4==0 else (18, 29, 39), self.point(0,iy/4), self.point(a.width,iy/4))
        if observation.pose is not None:
            x, y, yaw = observation.pose
            co, si = math.cos(yaw), math.sin(yaw)
            ox, oy = x+co*l.x-si*l.y, y+si*l.x+co*l.y
            center, half = yaw+math.radians(l.yaw_deg), math.radians(l.horizontal_deg)/2
            self.layer.fill((0,0,0,0))
            self.layer.set_clip(world)
            fan = [self.point(ox,oy)] + [self.point(ox+l.maximum*math.cos(center-half+2*half*i/64), oy+l.maximum*math.sin(center-half+2*half*i/64)) for i in range(65)]
            if l.enabled:
                pygame.draw.polygon(self.layer, (*self.TEAL, 13), fan)
                pygame.draw.lines(self.layer, (*self.TEAL, 65), True, fan, 1)
            self.screen.blit(self.layer, (0,0))
            if observation.lidar:
                self.screen.set_clip(world)
                for col in range(64):
                    idx = 3*64+col
                    mm = observation.lidar.values_mm[idx]
                    if l.minimum*1000 <= mm <= l.maximum*1000:
                        dx,dy,dz = self.rays[idx]
                        end = self.point(ox+(co*dx-si*dy)*mm/1000, oy+(si*dx+co*dy)*mm/1000)
                        pygame.draw.circle(self.screen, (80,176,152), end, 2)
                self.screen.set_clip(None)
        self.screen.set_clip(world.inflate(14,14))
        for b in a.boxes():
            self.rect_body(b.x+0.025, b.y-0.025, b.length,b.width,0,(4,8,13))
            self.rect_body(b.x,b.y,b.length,b.width,0,(61,76,91))
        self.screen.set_clip(None)
        if observation.pose is not None:
            x,y,yaw = observation.pose
            self.car((x+math.cos(yaw)*c.wheelbase/2,y+math.sin(yaw)*c.wheelbase/2,yaw), c.length,c.width,self.TEAL,math.radians(command.steering*c.max_steering_deg),"01")
        if observation.opponent is not None:
            self.car(observation.opponent,a.opponent_length,a.opponent_width,self.AMBER,label="02")
        pygame.draw.rect(self.screen, self.LINE, world, 1)
        self.text("N", world.centerx-5, world.y-28, self.MUTED, self.small)
        self.text("01 / VEHICLE", 594, 220, self.TEAL, self.small)
        hero = pygame.transform.rotozoom(self.vehicle(self.TEAL, math.radians(command.steering*c.max_steering_deg)),90,0.86)
        self.screen.blit(hero,hero.get_rect(center=(745,366)))
        self.text(f"{command.steering*c.max_steering_deg:+.0f}°", 704, 498, font=self.large)
        self.text("STEERING", 704, 536, self.MUTED, self.small)
        if observation.pose is not None:
            self.text(f"X  {observation.pose[0]:.2f} M", 594, 603, self.MUTED, self.small)
            self.text(f"Y  {observation.pose[1]:.2f} M", 762, 603, self.MUTED, self.small)
        self.text("01", 594, 650, self.TEAL, self.small)
        self.text("02", 670, 650, self.AMBER, self.small)
        self.text("OBSTACLES", 752, 650, self.MUTED, self.small)

        compass = sensors.get("compass")
        tag, tint = self.status(compass,now)
        self.panel((956,164,220,164),"HEADING",tag)
        heading = compass.values.get("heading_deg") if compass else None
        cx,cy = 1015,260
        pygame.draw.circle(self.screen,self.LINE,(cx,cy),39,1)
        for angle in range(0,360,30):
            t=math.radians(angle)
            pygame.draw.line(self.screen,self.MUTED,(cx+33*math.sin(t),cy-33*math.cos(t)),(cx+38*math.sin(t),cy-38*math.cos(t)),1)
        if heading is not None:
            t=math.radians(heading)
            pygame.draw.line(self.screen,self.TEAL,(cx,cy),(cx+28*math.sin(t),cy-28*math.cos(t)),3)
            self.text(f"{heading:.0f}°",1064,245,font=self.large)
        else: self.text("—",1064,245,font=self.large)
        battery = sensors.get("battery")
        tag,tint = self.status(battery,now,10)
        self.panel((1192,164,224,164),"BATTERY",tag)
        percent=battery.values.get("percent") if battery else None
        self.text(f"{percent:.0f}%" if percent is not None else "—",1214,218,font=self.metric)
        pygame.draw.rect(self.screen,self.LINE,(1214,290,178,7),border_radius=3)
        if percent is not None:
            pygame.draw.rect(self.screen,self.TEAL,(1214,290,round(178*percent/100),7),border_radius=3)
        imu=sensors.get("imu")
        tag,tint=self.status(imu,now)
        self.panel((956,348,460,132),"ACCELERATION / MG",tag)
        impact = sensors.get("bump")
        if impact and 0 <= now-impact.received_at < 1:
            self.text("IMPACT",1210,366,self.AMBER,self.small)
        if imu and "accel_mg" in imu.values:
            vals=imu.values["accel_mg"]
            if self.imu_key!=imu.received_at:
                self.imu_history.append(vals)
                self.imu_key=imu.received_at
            for index,(name,col) in enumerate(zip("XYZ",(self.TEAL,self.BLUE,self.AMBER))):
                self.text(f"{name}  {vals[index]:+d}",978+index*138,390,col,self.small)
            for index,col in enumerate((self.TEAL,self.BLUE,self.AMBER)):
                points=[(978+i*414/99,450-max(-2000,min(2000,v[index]))/2000*26) for i,v in enumerate(self.imu_history)]
                if len(points)>1: pygame.draw.aalines(self.screen,col,False,points)
        vision=sensors.get("vision")
        tag,tint=self.status(vision,now)
        self.panel((956,500,460,250),"VISION",tag)
        viewport=pygame.Rect(978,542,250,188)
        pygame.draw.rect(self.screen,(9,16,23),viewport,border_radius=6)
        pygame.draw.line(self.screen,self.LINE,(1103,542),(1103,730),1)
        pygame.draw.line(self.screen,self.LINE,(978,636),(1228,636),1)
        blocks=vision.values.get("blocks",[]) if vision else []
        arrows=vision.values.get("arrows",[]) if vision else []
        if vision and "width" in vision.values:
            self.screen.set_clip(viewport)
            for block in blocks:
                bx,by,bw,bh,identity,confidence=block
                rect=pygame.Rect(978+(bx-bw/2)/vision.values["width"]*250,542+(by-bh/2)/vision.values["height"]*188,bw/vision.values["width"]*250,bh/vision.values["height"]*188)
                pygame.draw.rect(self.screen,self.TEAL,rect,1)
                self.text(f"{identity:02d}",rect.x+3,rect.y+3,self.TEAL,self.small)
            for x0,y0,x1,y1,identity,confidence in arrows:
                start=(978+x0/vision.values["width"]*250,542+y0/vision.values["height"]*188)
                end=(978+x1/vision.values["width"]*250,542+y1/vision.values["height"]*188)
                pygame.draw.line(self.screen,self.AMBER,start,end,2)
                angle=math.atan2(end[1]-start[1],end[0]-start[0])
                for side in (-0.5,0.5):
                    pygame.draw.line(self.screen,self.AMBER,end,(end[0]-10*math.cos(angle+side),end[1]-10*math.sin(angle+side)),2)
            self.screen.set_clip(None)
        self.text(f"{len(blocks)+len(arrows):02d}",1252,560,font=self.metric)
        self.text("DETECTIONS",1252,617,self.MUTED,self.small)
        self.text(" / ".join(str(b[4]) for b in blocks) or "—",1252,661,self.TEAL,self.small)

        frame=observation.lidar
        fresh = frame is not None and frame.fresh(now, l.stale_seconds)
        self.panel((24,728,912,208),"LIDAR / SENSOR RANGE", f"{l.hz} HZ" if fresh else "STALE" if frame else "OFFLINE")
        self.text('gray: no return   pink X: out of range', 350, 747, self.MUTED, self.small)
        if frame is not None and (frame.frame_id,frame.received_at)!=self.heatmap_key:
            for index,mm in enumerate(frame.values_mm):
                col=range_color(mm, l.minimum*1000, l.maximum*1000)
                pygame.draw.rect(self.heatmap,col,((index%64)*12,(index//64)*10,12,10))
                if mm and not l.minimum*1000 <= mm <= l.maximum*1000:
                    x, y = (index%64)*12, (index//64)*10
                    pygame.draw.line(self.heatmap, self.INK, (x+3,y+2), (x+8,y+7))
                    pygame.draw.line(self.heatmap, self.INK, (x+8,y+2), (x+3,y+7))
            match = closest(frame.values_mm, l.minimum*1000, l.maximum*1000)
            self.nearest = match[0]/1000 if match else None
            self.closest_cell = divmod(match[1], 64) if match else None
            self.heatmap_key=(frame.frame_id,frame.received_at)
        if frame is None:
            self.heatmap.fill(range_color(0))
            self.heatmap_key = self.nearest = self.closest_cell = None
        self.heatmap.set_alpha(255 if fresh or frame is None else 90)
        self.screen.blit(self.heatmap,(96,785))
        self.text("r0",61,785,self.MUTED,self.small)
        self.text("r7",61,852,self.MUTED,self.small)
        self.text("c0",96,766,self.MUTED,self.small)
        self.text("c63",835,766,self.MUTED,self.small)
        for i in range(200):
            pygame.draw.line(self.screen, range_color((l.minimum+(l.maximum-l.minimum)*i/199)*1000,
                l.minimum*1000, l.maximum*1000), (310+i,769), (310+i,776))
        self.text(f"{l.minimum:g}m near",234,766,self.MUTED,self.small)
        self.text(f"{l.maximum:g}m far",520,766,self.MUTED,self.small)
        self.selected = self.lidar_cell(pygame.mouse.get_pos()) if frame and pygame.mouse.get_focused() else None
        if self.selected is not None:
            row, col = self.selected
            pygame.draw.rect(self.screen, self.INK, (96+col*12,785+row*10,12,10), 2)
            mm = frame.values_mm[row*64+col]
            label = f"CELL r{row} c{col}   {distance(mm, l.minimum*1000, l.maximum*1000)}"
            self.text(label,46,908,self.INK if fresh else self.AMBER,self.small)
        else:
            self.text("Hover a cell for row, column and distance" if frame else "Waiting for LiDAR data",46,908,self.MUTED,self.small)
        age=now-frame.received_at if frame else None
        nearest = f"NEAREST IN RANGE {self.nearest:.3f} m  r{self.closest_cell[0]} c{self.closest_cell[1]}" if self.nearest is not None else "NEAREST IN RANGE —"
        self.text(nearest,46,880,self.TEAL if fresh else self.MUTED)
        self.text(f"{age*1000:.0f} MS" if age is not None else "OFFLINE",730,880,self.MUTED,self.small)
        self.text("LIVE" if fresh else "STALE" if frame else "OFFLINE",828,880,self.TEAL if fresh else self.AMBER,self.small)
        self.panel((956,770,460,166),"DRIVE")
        motion = sensors.get("motion")
        speed = abs(motion.values["velocity_mps"]) if motion else None
        self.text(f"{speed:.2f}" if speed is not None else "—",978,810,font=self.metric)
        self.text("M/S",1104,841,self.MUTED,self.small)
        self.text("R" if command.throttle<0 else "D" if command.throttle>0 else "N",1318,807,self.TEAL,font=self.metric)
        pygame.draw.rect(self.screen,self.LINE,(978,899,416,5),border_radius=2)
        pygame.draw.rect(self.screen,self.TEAL,(978,899,round(416*abs(command.throttle)),5),border_radius=2)
        viewport = self.viewport()
        output=pygame.transform.smoothscale(self.screen,viewport.size)
        self.window.fill(self.BG)
        self.window.blit(output,viewport)
        pygame.display.flip()

    def save(self,path):
        pygame.image.save(self.screen,str(path))

    def close(self):
        pygame.quit()


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
