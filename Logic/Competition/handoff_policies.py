"""Decisiones independientes para transferir información entre robots."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from .environment import Observation


class HandoffPolicy(ABC):
    @abstractmethod
    def decide(self, obs: Observation, options) -> int | None:
        """Devuelve el id del receptor o ``None`` para continuar hacia base."""


class ClosestProgressHandoffPolicy(HandoffPolicy):
    """Baseline: receptor conectado más cercano que ya está más cerca de base."""

    def decide(self, obs, options):
        own_home = float(np.linalg.norm(obs.pose-obs.base_pose))
        candidates = []
        for robot_id in obs.connected_robot_ids:
            poses = obs.pose_lists_of_others.get(f"robot{robot_id}")
            if poses is None or not len(poses):
                continue
            peer = np.asarray(poses)[-1]
            if np.linalg.norm(peer-obs.base_pose) < own_home:
                candidates.append((np.linalg.norm(peer-obs.pose), robot_id))
        return min(candidates)[1] if candidates else None


class PayloadProgressHandoffPolicy(HandoffPolicy):
    """Entrega sólo una carga útil a un receptor con progreso verificable."""

    def __init__(self, minimum_payload: int = 300,
                 minimum_progress_m: float = 1.5,
                 maximum_away_motion_m: float = .5) -> None:
        self.minimum_payload = int(minimum_payload)
        self.minimum_progress_m = float(minimum_progress_m)
        self.maximum_away_motion_m = float(maximum_away_motion_m)

    def decide(self, obs, options):
        payload = int(np.count_nonzero(obs.unreported_mask))
        if payload < self.minimum_payload:
            return None
        ppm = float(options.pixel_per_meter)
        own_home = float(np.linalg.norm(obs.pose-obs.base_pose))
        best = None
        for robot_id in obs.connected_robot_ids:
            poses = obs.pose_lists_of_others.get(f"robot{robot_id}")
            if poses is None or not len(poses):
                continue
            poses = np.asarray(poses)
            peer = poses[-1]
            peer_home = float(np.linalg.norm(peer-obs.base_pose))
            progress = (own_home-peer_home)/ppm
            if progress < self.minimum_progress_m:
                continue
            meeting = float(np.linalg.norm(peer-obs.pose))/ppm
            if meeting > options.comm_range:
                continue
            away_motion = 0.0
            if len(poses) >= 2:
                previous_home = float(np.linalg.norm(poses[-2]-obs.base_pose))
                away_motion = (peer_home-previous_home)/ppm
            if away_motion > self.maximum_away_motion_m:
                continue
            # Favorece progreso hacia base y penaliza un receptor lejano.
            score = 2.0*progress-meeting-2.0*max(away_motion, 0.0)
            if best is None or score > best[0]:
                best = (score, int(robot_id))
        return None if best is None else best[1]


def _peer_states(obs: Observation, options):
    """Datos cinemáticos comunes de receptores conectados."""
    own_home = float(np.linalg.norm(obs.pose-obs.base_pose))
    ppm = float(options.pixel_per_meter)
    for robot_id in obs.connected_robot_ids:
        poses = obs.pose_lists_of_others.get(f"robot{robot_id}")
        if poses is None or not len(poses):
            continue
        poses = np.asarray(poses)
        peer = poses[-1]
        peer_home = float(np.linalg.norm(peer-obs.base_pose))
        previous_home = (float(np.linalg.norm(poses[-2]-obs.base_pose))
                         if len(poses) >= 2 else peer_home)
        yield int(robot_id), peer, (own_home-peer_home)/ppm, \
            (previous_home-peer_home)/ppm, float(np.linalg.norm(peer-obs.pose))/ppm


class TimeSavingHandoffPolicy(HandoffPolicy):
    """Transfiere sólo si reduce de forma material el tiempo hasta base."""

    def __init__(self, minimum_payload: int = 80, minimum_saving_m: float = .5) -> None:
        self.minimum_payload = int(minimum_payload)
        self.minimum_saving_m = float(minimum_saving_m)

    def decide(self, obs, options):
        payload = int(np.count_nonzero(obs.unreported_mask))
        if payload < self.minimum_payload:
            return None
        best = None
        for robot_id, peer, progress, toward_base, meeting in _peer_states(obs, options):
            intent = obs.intents_of_others.get(f"robot{robot_id}")
            intent_progress = 0.0
            if intent is not None and len(intent):
                endpoint = np.asarray(intent)[-1]
                intent_progress = (np.linalg.norm(peer-obs.base_pose)
                                   - np.linalg.norm(endpoint-obs.base_pose)) \
                                  / options.pixel_per_meter
            courier_progress = max(toward_base, intent_progress)
            # El ahorro se descuenta por el coste/riesgo de coordinar el encuentro.
            net_saving = progress-.35*meeting
            if (progress < .5 or net_saving < self.minimum_saving_m
                    or courier_progress < -.1):
                continue
            score = payload*(net_saving+.5*courier_progress)/(1.0+meeting)
            if best is None or score > best[0]:
                best = (score, robot_id)
        return None if best is None else best[1]


class ReturningCourierHandoffPolicy(HandoffPolicy):
    """Prefiere receptores cuyo movimiento o intent ya apunta hacia base."""

    def __init__(self, minimum_payload: int = 100) -> None:
        self.minimum_payload = int(minimum_payload)

    def decide(self, obs, options):
        payload = int(np.count_nonzero(obs.unreported_mask))
        if payload < self.minimum_payload:
            return None
        best = None
        for robot_id, peer, progress, toward_base, meeting in _peer_states(obs, options):
            intent = obs.intents_of_others.get(f"robot{robot_id}")
            intent_progress = 0.0
            if intent is not None and len(intent):
                endpoint = np.asarray(intent)[-1]
                intent_progress = (np.linalg.norm(peer-obs.base_pose)
                                   - np.linalg.norm(endpoint-obs.base_pose)) \
                                  / options.pixel_per_meter
            if progress < .5 or max(toward_base, intent_progress) < -.1:
                continue
            score = 1.5*progress+max(toward_base, intent_progress)-.5*meeting
            if best is None or score > best[0]:
                best = (score, robot_id)
        return None if best is None else best[1]


class LinkQualityHandoffPolicy(HandoffPolicy):
    """Combina ahorro hacia base con oclusión estimada del enlace."""

    def __init__(self, minimum_payload: int = 100) -> None:
        self.minimum_payload = int(minimum_payload)

    @staticmethod
    def _wall_count(obs, peer) -> int:
        count = max(int(np.max(np.abs(peer-obs.pose)))+1, 2)
        rows = np.rint(np.linspace(obs.pose[0], peer[0], count)).astype(int)
        cols = np.rint(np.linspace(obs.pose[1], peer[1], count)).astype(int)
        return int(np.count_nonzero(obs.combined_obs_map[rows, cols] == 1))

    def decide(self, obs, options):
        payload = int(np.count_nonzero(obs.unreported_mask))
        if payload < self.minimum_payload:
            return None
        best = None
        attenuation = float(getattr(options, "attenuation_constant", 10.0))
        for robot_id, peer, progress, toward_base, meeting in _peer_states(obs, options):
            intent = obs.intents_of_others.get(f"robot{robot_id}")
            intent_progress = 0.0
            if intent is not None and len(intent):
                endpoint = np.asarray(intent)[-1]
                intent_progress = (np.linalg.norm(peer-obs.base_pose)
                                   - np.linalg.norm(endpoint-obs.base_pose)) \
                                  / options.pixel_per_meter
            courier_progress = max(toward_base, intent_progress)
            if (progress < .5 or courier_progress < -.1
                    or meeting > options.comm_range):
                continue
            wall_penalty = self._wall_count(obs, peer)*attenuation/10.0
            score = (2.0*progress+courier_progress
                     -.4*meeting-wall_penalty)
            if score > 0 and (best is None or score > best[0]):
                best = (score, robot_id)
        return None if best is None else best[1]


def make_handoff_policy(name: str) -> HandoffPolicy:
    policies = {
        "closest_progress": ClosestProgressHandoffPolicy,
        "payload_progress": PayloadProgressHandoffPolicy,
        "time_saving": TimeSavingHandoffPolicy,
        "returning_courier": ReturningCourierHandoffPolicy,
        "link_quality": LinkQualityHandoffPolicy,
    }
    try:
        return policies[name]()
    except KeyError as error:
        raise ValueError(f"Handoff policy desconocida: {name}") from error
