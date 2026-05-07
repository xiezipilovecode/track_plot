import logging

from .carla_compat import require_carla

from .env_utils import _get_bool_from_env, _get_float_from_env


logger = logging.getLogger(__name__)


def _maybe_generate_opendrive_world(client, xodr_path):
    """Optionally generate OpenDRIVE world in CARLA server."""

    preserve_existing = _get_bool_from_env("TP_PRESERVE_EXISTING_WORLD", True)
    if preserve_existing:
        logger.info(
            "TP_PRESERVE_EXISTING_WORLD=1: skip generate_opendrive_world; using current server world"
        )
        return client.get_world()

    use_generate = _get_bool_from_env("TP_USE_GENERATE_OPENDRIVE_WORLD", True)
    if not use_generate:
        return client.get_world()

    if not xodr_path or not str(xodr_path).lower().endswith(".xodr"):
        return client.get_world()

    try:
        with open(xodr_path, "r", encoding="utf-8") as f:
            xodr_content = f.read()
    except OSError as e:
        logger.warning(
            "Failed to read XODR_PATH=%s: %s; falling back to get_world()", xodr_path, e
        )
        return client.get_world()

    try:
        carla = require_carla()
        params = carla.OpendriveGenerationParameters()
        params.vertex_distance = _get_float_from_env("TP_OPENDRIVE_VERTEX_DISTANCE", 2.0)
        params.max_road_length = _get_float_from_env("TP_OPENDRIVE_MAX_ROAD_LENGTH", 50.0)
        params.smooth_junctions = _get_bool_from_env("TP_OPENDRIVE_SMOOTH_JUNCTIONS", True)
        params.enable_mesh_visibility = _get_bool_from_env(
            "TP_OPENDRIVE_ENABLE_MESH_VISIBILITY", True
        )
        params.enable_pedestrian_navigation = _get_bool_from_env(
            "TP_OPENDRIVE_ENABLE_PEDESTRIAN_NAVIGATION", False
        )
        params.additional_width = _get_float_from_env("TP_OPENDRIVE_ADDITIONAL_WIDTH", 0.0)
        params.wall_height = _get_float_from_env("TP_OPENDRIVE_WALL_HEIGHT", 1.0)
    except Exception as e:
        logger.warning("Failed to build OpendriveGenerationParameters: %s; using defaults", e)
        carla = require_carla()
        params = carla.OpendriveGenerationParameters()

    logger.info(
        "Generating OpenDRIVE world from %s (len=%d) vertex_distance=%.3f max_road_length=%.3f smooth_junctions=%s",
        xodr_path,
        len(xodr_content),
        float(getattr(params, "vertex_distance", 0.0)),
        float(getattr(params, "max_road_length", 0.0)),
        bool(getattr(params, "smooth_junctions", False)),
    )

    try:
        preserve_weather = _get_bool_from_env("TP_PRESERVE_WEATHER", True)
        weather_before = None
        if preserve_weather:
            try:
                weather_before = client.get_world().get_weather()
            except Exception as e:
                logger.warning(
                    "Failed to read current weather before world generation: %s", e
                )

        world = client.generate_opendrive_world(xodr_content, params)

        if preserve_weather and weather_before is not None:
            try:
                world.set_weather(weather_before)
                logger.info(
                    "Restored weather after generate_opendrive_world (TP_PRESERVE_WEATHER=1)"
                )
            except Exception as e:
                logger.warning("Failed to restore weather after world generation: %s", e)

        return world
    except Exception as e:
        logger.error(
            "generate_opendrive_world failed for %s: %s; falling back to get_world()",
            xodr_path,
            e,
        )
        return client.get_world()
