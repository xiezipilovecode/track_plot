from __future__ import annotations

from typing import Iterable


def destroy_actors(actors: Iterable[object]):
    for actor in actors:
        try:
            if actor is not None and actor.is_alive:
                actor.destroy()
        except Exception:
            continue
