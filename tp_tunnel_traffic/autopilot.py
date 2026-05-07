from __future__ import annotations


def configure_autopilot(vehicle, tm, config):
    vehicle.set_autopilot(True, tm.get_port())
    try:
        tm.ignore_lights_percentage(vehicle, 100.0 if config.tm_ignore_lights else 0.0)
        tm.ignore_signs_percentage(vehicle, 100.0 if config.tm_ignore_signs else 0.0)
        tm.auto_lane_change(vehicle, bool(config.tm_auto_lane_change))
        tm.distance_to_leading_vehicle(vehicle, float(config.tm_follow_distance))
        tm.vehicle_percentage_speed_difference(vehicle, float(config.tm_speed_diff_percent))
    except Exception:
        pass


def configure_traffic_manager(client, config):
    tm = client.get_trafficmanager(int(config.tm_port))
    try:
        tm.set_synchronous_mode(bool(config.sync_mode))
    except Exception:
        pass
    return tm
