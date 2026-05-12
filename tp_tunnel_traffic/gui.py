import queue
from typing import Any

import os

pygame: Any = None
np: Any = None
HAS_PYGAME = False
try:
    os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
    import pygame  # type: ignore[reportMissingImports]
    import numpy as np  # type: ignore[reportMissingImports]
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False


class TunnelTrafficGUI:
    def __init__(self, world, vehicle, config):
        self.enabled = config.gui_enable and HAS_PYGAME
        if config.gui_enable and not HAS_PYGAME:
            print("WARNING: TT_GUI_ENABLE=1 but pygame or numpy is not installed. GUI disabled.")
        if not self.enabled:
            return

        self.width = 1050
        self.height = 680
        self.panel_h = 86
        
        pygame.init()
        self.display = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Tunnel Traffic Control Panel")

        self.world = world
        self.vehicle = vehicle
        self.image_queue = queue.Queue()
        
        # Spawn camera for GUI (display-only; transform由 main 统一计算)
        bp_lib = world.get_blueprint_library()
        camera_bp = bp_lib.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(self.width))
        camera_bp.set_attribute("image_size_y", str(self.height - self.panel_h))
        camera_bp.set_attribute("fov", "90")
        
        # Initially attach to world (will update transform manually)
        initial_tf = vehicle.get_transform() if vehicle is not None else world.get_spectator().get_transform()
        self.camera = world.spawn_actor(camera_bp, initial_tf)
        self.camera.listen(self.image_queue.put)
        
        self.surface = None

        # Buttons definition
        self.buttons = [
            {"label": "Ego View", "rect": pygame.Rect(18, 18, 110, 36), "action": "ego", "group": "view"},
            {"label": "Overview", "rect": pygame.Rect(140, 18, 120, 36), "action": "overview", "group": "view"},
            {"label": "Collect Selected", "rect": pygame.Rect(272, 18, 170, 36), "action": "toggle_collect_target", "group": "collect"},
            {"label": "Record Video", "rect": pygame.Rect(452, 18, 170, 36), "action": "toggle_video_record", "group": "video"},
            {"label": "Reset Cam", "rect": pygame.Rect(632, 18, 120, 36), "action": "overview_reset", "group": "overview"},
            {"label": "Yaw -", "rect": pygame.Rect(762, 18, 86, 36), "action": "yaw_left", "group": "yaw"},
            {"label": "Reset", "rect": pygame.Rect(858, 18, 86, 36), "action": "yaw_reset", "group": "yaw"},
            {"label": "Yaw +", "rect": pygame.Rect(954, 18, 86, 36), "action": "yaw_right", "group": "yaw"},
        ]
        self.font = pygame.font.SysFont(None, 26)
        self.small_font = pygame.font.SysFont(None, 20)
        self.pending_action = None
        self._pending_overview_action = None
        self._state_provider = None
        self.proxy_rows = []

        # Overview free-fly state is driven by keyboard each tick.
        self._overview_accum_move = (0.0, 0.0, 0.0)

        # Mouse look state for overview.
        self._overview_mouse_look_enabled = False

    def update_transform(self, transform):
        if not self.enabled or not self.camera:
            return
        try:
            self.camera.set_transform(transform)
        except Exception:
            pass

    def update_state_provider(self, provider):
        self._state_provider = provider

    def tick(self):
        if not self.enabled:
            return None

        state = {}
        if self._state_provider is not None:
            state = self._state_provider() or {}
        view_mode = state.get("view_mode", "ego")

        overview_actions: list[str] = []
        look_dx = 0.0
        look_dy = 0.0
        
        # Process events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return "quit"
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    for b in self.buttons:
                        if b["rect"].collidepoint(event.pos):
                            self.pending_action = b["action"]
                    for row in self.proxy_rows:
                        if row["rect"].collidepoint(event.pos):
                            self.pending_action = f"select_proxy:{row['actor_id']}"

                # Overview mouse-look: hold RMB and drag.
                elif event.button == 3 and view_mode == "overview":
                    self._overview_mouse_look_enabled = True

                # Mouse wheel zoom (pygame 1.x)
                elif view_mode == "overview" and event.button in (4, 5):
                    dz = 1.0 if event.button == 4 else -1.0
                    overview_actions.append(f"overview_zoom:{dz:.3f}")

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3:
                    self._overview_mouse_look_enabled = False

            elif event.type == pygame.MOUSEMOTION:
                if view_mode == "overview" and self._overview_mouse_look_enabled:
                    try:
                        dx, dy = event.rel
                        look_dx += float(dx)
                        look_dy += float(dy)
                    except Exception:
                        pass

            elif event.type == getattr(pygame, "MOUSEWHEEL", -1):
                if view_mode == "overview":
                    try:
                        y = float(getattr(event, "y", 0.0))
                        if abs(y) > 1e-6:
                            overview_actions.append(f"overview_zoom:{y:.3f}")
                    except Exception:
                        pass

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_r, pygame.K_HOME):
                    overview_actions.append("overview_reset")

        # Consume image queue to get the latest frame
        latest_image = None
        while not self.image_queue.empty():
            latest_image = self.image_queue.get()
            
        if latest_image is not None:
            # Convert carla raw data to pygame surface
            array = np.frombuffer(latest_image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.height - self.panel_h, self.width, 4))
            array = array[:, :, :3]      # BGRA to BGR
            array = array[:, :, ::-1]    # BGR to RGB
            array = np.ascontiguousarray(array)
            # make_surface expects shape (width, height, 3)
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        if self.surface is not None:
            self.display.blit(self.surface, (0, self.panel_h))

        panel_rect = pygame.Rect(0, 0, self.width, self.panel_h)
        pygame.draw.rect(self.display, (20, 24, 34), panel_rect)
        pygame.draw.line(self.display, (70, 80, 96), (0, self.panel_h - 1), (self.width, self.panel_h - 1), 2)

        title = self.font.render("Tunnel Traffic Control Panel", True, (245, 245, 245))
        self.display.blit(title, (18, 56))

        collect_proxy_enabled = bool(state.get("collect_proxy_enabled", False))
        selected_proxy_actor_id = state.get("selected_proxy_actor_id")
        video_recording = bool(state.get("video_recording", False))
        target_text = "ego" if selected_proxy_actor_id is None else f"proxy#{int(selected_proxy_actor_id)}"
        mode_text = (
            f"View: {state.get('view_mode', 'ego')}   "
            f"Yaw: {float(state.get('yaw_offset_deg', 0.0)):+.1f}°   "
            f"Target: {target_text}   "
            f"Collect: {'ON' if collect_proxy_enabled else 'OFF'}   "
            f"Video: {'ON' if video_recording else 'OFF'}"
        )
        mode_surf = self.small_font.render(mode_text, True, (210, 220, 235))
        self.display.blit(mode_surf, (18, 59))

        proxies = list(state.get("proxies", []))
        selected_proxy_actor_id = state.get("selected_proxy_actor_id")

        # Draw buttons
        for b in self.buttons:
            current = state
            active = False
            if b["group"] == "view":
                active = current.get("view_mode") == b["action"]
            elif b["group"] == "collect":
                active = bool(current.get("collect_proxy_enabled", False)) and current.get("selected_proxy_actor_id") is not None
            elif b["group"] == "video":
                active = bool(current.get("video_recording", False))
            elif b["group"] == "overview":
                active = False
            elif b["group"] == "yaw":
                active = False
            fill = (46, 90, 140) if active else (54, 58, 68)
            border = (120, 170, 245) if active else (180, 184, 192)
            pygame.draw.rect(self.display, fill, b["rect"], border_radius=8)
            pygame.draw.rect(self.display, border, b["rect"], 2, border_radius=8)
            text = self.font.render(b["label"], True, (255, 255, 255))
            text_rect = text.get_rect(center=b["rect"].center)
            self.display.blit(text, text_rect)

        # Proxy list panel
        self.proxy_rows = []
        list_x = 18
        list_y = self.panel_h + 14
        list_w = 260
        row_h = 28
        title = self.small_font.render("Proxies", True, (220, 230, 245))
        self.display.blit(title, (list_x, list_y - 18))
        for idx, proxy in enumerate(proxies[:12]):
            rect = pygame.Rect(list_x, list_y + idx * (row_h + 6), list_w, row_h)
            self.proxy_rows.append({"rect": rect, "actor_id": proxy.get("actor_id")})
            alive = bool(proxy.get("alive"))
            active = proxy.get("actor_id") == selected_proxy_actor_id
            fill = (63, 120, 74) if alive else (72, 72, 72)
            if active:
                fill = (52, 96, 150)
            border = (160, 220, 180) if alive else (110, 110, 110)
            if active:
                border = (130, 190, 255)
            pygame.draw.rect(self.display, fill, rect, border_radius=6)
            pygame.draw.rect(self.display, border, rect, 2, border_radius=6)
            label = f"#{proxy.get('actor_id')} L{proxy.get('lane_id')} W{proxy.get('waypoint_index')}"
            if not alive:
                label += " dead"
            text = self.small_font.render(label, True, (255, 255, 255))
            self.display.blit(text, (rect.x + 8, rect.y + 5))

        pygame.display.flip()

        # Emit overview free-fly movement deltas based on key state.
        if view_mode == "overview":
            keys = pygame.key.get_pressed()
            move_x = 0.0  # strafe
            move_y = 0.0  # forward/back
            move_z = 0.0  # up/down
            speed = 1.0
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                speed = 2.5
            if keys[pygame.K_w]:
                move_y += speed
            if keys[pygame.K_s]:
                move_y -= speed
            if keys[pygame.K_a]:
                move_x -= speed
            if keys[pygame.K_d]:
                move_x += speed
            if keys[pygame.K_q]:
                move_z += speed
            if keys[pygame.K_e]:
                move_z -= speed
            if abs(move_x) > 1e-6 or abs(move_y) > 1e-6 or abs(move_z) > 1e-6:
                overview_actions.append(f"overview_move:{move_x:.3f},{move_y:.3f},{move_z:.3f}")

        if view_mode == "overview" and (abs(look_dx) > 1e-6 or abs(look_dy) > 1e-6):
            overview_actions.append(f"overview_look:{look_dx:.3f},{look_dy:.3f}")

        if overview_actions:
            self._pending_overview_action = ";".join(overview_actions)

        action = self.pending_action or self._pending_overview_action
        self.pending_action = None
        self._pending_overview_action = None
        return action

    def destroy(self):
        if not self.enabled:
            return
        if self.camera:
            try:
                self.camera.stop()
                self.camera.destroy()
            except Exception:
                pass
        pygame.quit()
