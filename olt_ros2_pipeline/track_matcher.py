from collections import namedtuple
import numpy as np
import numpy.typing as npt
import pinocchio as pin
from typing import Dict, List

from geometry_msgs.msg import Point, Pose, Quaternion
from vision_msgs.msg import Detection2DArray

TrackPose = namedtuple("TrackPose", ["track_id", "class_id", "transformation"])
NewDetection = namedtuple("NewDetection", ["idx", "class_id", "transformation"])
TrackDetectionCloseness = namedtuple(
    "TrackDetectionCloseness", ["idx", "track_id", "cost", "transformation"]
)


class TrackMatcher:
    """Class performing track assignment."""

    def __init__(
        self,
        symmetries: Dict[str, npt.NDArray[np.float64]],
        track_max_dist: float = 0.05,
        distance_cost_weight: float = 1.0,
        angle_cost_weight: float = 1.0,
    ) -> None:
        """Initializes TrackMatcher object.

        :param symmetries: Dictionary with symmetries of object class IDs.
        :type symmetries: Dict[str, npt.NDArray[np.float64]]
        :param track_max_dist: Maximum distance between detection
            and track in meters, defaults to 0.05.
        :type track_max_dist: float, optional
        :param distance_cost_weight: Weight applied to distance between detection
            and track when computing cost associated to the given pair, defaults to 1.0.
        :type distance_cost_weight: float, optional
        :param angle_cost_weight: Weight applied to angles between detection
            and track when computing cost associated to the given pair, defaults to 1.0.
        :type angle_cost_weight: float, optional
        """
        self.symmetries = symmetries
        self._tracked_objects = []
        self._object_track_counter = {}
        self.track_max_dist = track_max_dist
        self.distance_cost_weight = distance_cost_weight
        self.angle_cost_weight = angle_cost_weight
        self._is_seeded = False

    @property
    def is_seeded(self) -> bool:
        """Return information if the track assigner is seeded.

        :return: State of the variable ``self._is_seeded``.
        :rtype: bool
        """
        return self._is_seeded

    def seed_track(self, detections: Detection2DArray) -> Detection2DArray:
        """Initializes the track matcher with initial detection.

        :param detections: ROS message with detection to initialize track matcher.
        :type detections: vision_msgs.msg.Detection2DArray
        :return: The initial message input with applied tracks.
        :rtype: vision_msgs.msg.Detection2DArray
        """
        # Generate empty dictionary with expected IDs of classes
        for det in detections.detections:
            self._object_track_counter[det.results[0].hypothesis.class_id] = 0

        # Assign track IDs
        for i in range(len(detections.detections)):
            class_id = detections.detections[i].results[0].hypothesis.class_id
            object_idx = self._object_track_counter[class_id]
            detections.detections[i].id = f"{class_id}_{object_idx}"
            self._object_track_counter[class_id] += 1

        self._is_seeded = True

        return detections

    def _pose_to_se3(self, msg: Pose) -> pin.SE3:
        """Converts ROS Pose message to Pinocchio object.

        :param msg: Pose to be converted to SE3.
        :type msg: geometry_msgs.msg.Pose
        :return: SE3 object representing transformation
            associated with the ROS message.
        :rtype: pinocchio.SE3
        """
        p = msg.position
        o = msg.orientation
        return pin.XYZQUATToSE3([p.x, p.y, p.z, o.x, o.y, o.z, o.w])

    def _se3_to_pose(self, se3: pin.SE3) -> Pose:
        """Converts ROS Pose messagePinocchio object to ROS Pose message.

        :param msg: SE3 object to be converted into ROS message.
        :type msg: pinocchio.SE3
        :return: ROS message containing the transformation from SE3 object.
        :rtype: geometry_msgs.msg.Pose
        """
        pose_vec = pin.SE3ToXYZQUAT(se3)
        return Pose(
            position=Point(**dict(zip("xyz", pose_vec[:3]))),
            orientation=Quaternion(**dict(zip("xyzw", pose_vec[3:]))),
        )

    def _has_track_candidate(self, det: NewDetection, tracks: List[TrackPose]) -> bool:
        """Checks if there is any track within distance threshold.

        :param det: New detection to be compared to known tracks.
        :type det: NewDetection
        :param tracks: List of tracks to compare for closeness.
        :type tracks: List[TrackPose]
        :return: ``True`` is at least one track is close to the detection.
        :rtype: bool
        """
        return any(self._track_close_enough(det, t) for t in tracks)

    def _track_close_enough(self, det: NewDetection, track: TrackPose) -> bool:
        """Checks if specified track is close enough to the detection.

        :param det: New detection to be compared with the track.
        :type det: NewDetection
        :param track: Track used for comparison.
        :type track: TrackPose
        :return: ``True`` if distance between detection
            and track is smaller than threshold.
        :rtype: bool
        """
        return (
            np.linalg.norm(
                det.transformation.translation - track.transformation.translation
            )
            < self.track_max_dist
        )

    def _find_minimum_costs(
        self, det: NewDetection, tracks: List[TrackPose]
    ) -> List[TrackDetectionCloseness]:
        """Applies transformations associated to all symmetries of the detection.
        Computes cost between every symmetry of detection and each track that is
        close enough to it. Returns list costs associated to each track. Values
        for each track already apply symmetry that minimizes the cost.

        :param det: New detection of which symmetries will be compared to the track.
        :type det: NewDetection
        :param tracks: List of tracks to compare the detection to.
        :type tracks: List[TrackPose]
        :return: List of costs related to the tracks.
        :rtype: List[TrackDetectionCloseness]
        """

        def _cost(det: pin.SE3, track: TrackPose, idx: int) -> TrackDetectionCloseness:
            """Computes cost related to a difference between two tracks as a sum
            of distance between detections and their relative angle multiplied by
            related costs.

            :param det: SE3 pose of a new detection.
            :type det: pinocchio.SE3
            :param track: Considered track to be matched.
            :type track: TrackPose
            :param idx: Positional index of matched detection used to reconstruct
                order from the original list.
            :type idx: int
            :return: Object with information needed to reconstruct initial list
                of detections.
            :rtype: TrackDetectionCloseness
            """
            dist = (
                np.linalg.norm(det.translation - track.transformation.translation)
                * self.distance_cost_weight
            )
            angle = (
                np.linalg.norm(pin.log3(det.rotation * track.transformation.rotation.T))
                * self.angle_cost_weight
            )
            return TrackDetectionCloseness(idx, track.track_id, dist + angle, det)

        if det.class_id in self.symmetries and len(self.symmetries[det.class_id]) > 0:
            # Compute all permutations of detection and its symmetries
            det_symmetries = det.transformation.np @ self.symmetries[det.class_id]
            det_symmetries = [pin.SE3(sym) for sym in det_symmetries]
            # Add the initial symmetry to the list
            det_symmetries.append(det.transformation)
        else:
            det_symmetries = [det.transformation]

        return [
            min(
                [_cost(sym, track, det.idx) for sym in det_symmetries],
                key=lambda c: c.cost,
            )
            for track in tracks
            if self._track_close_enough(det, track)
        ]

    def update_tracked_objects(self, detections: Detection2DArray) -> None:
        """Updates internal list of tracks to match new detection and merges
        new detections with old ones in case they are of the same type and
        are close enough.

        :param detections: Detections used to match tracks.
        :type detections: vision_msgs.msg.Detection2DArray
        """
        self._tracked_objects = [
            TrackPose(
                det.id,
                det.results[0].hypothesis.class_id,
                self._pose_to_se3(det.results[0].pose.pose),
            )
            for det in detections.detections
        ]

    def match_tracks(self, detections: Detection2DArray) -> Detection2DArray:
        """Checks if new detections match with previously assigned tracks. If detections match,
        old track is assigned and symmetry closest to old track is applied to the new detection.
        If tracks don't match the detection has new track assigned.

        :param detections: New detection to be matched with old tracks.
        :type detections: vision_msgs.msg.Detection2DArray
        :raises RuntimeError: ``seed_track`` was not called before.
        :return: Detections after track assigning.
        :rtype: vision_msgs.msg.Detection2DArray
        """

        if not self._is_seeded:
            raise RuntimeError("Tracker was not seeded!")

        if len(detections.detections) == 0:
            self._tracked_objects = []
            return detections

        # Create set of all possible class IDs
        tracked_classes = {track.class_id for track in self._tracked_objects}
        # Convert ROS message into list of custom objects, while filtering those
        # of which class ID was not seen in a previous detection.
        valid_detections = [
            NewDetection(
                idx,
                det.results[0].hypothesis.class_id,
                self._pose_to_se3(det.results[0].pose.pose),
            )
            for idx, det in enumerate(detections.detections)
            if det.results[0].hypothesis.class_id in tracked_classes
        ]

        # Create set of class ID of newly tracked objects
        valid_detections_classes = {det.class_id for det in valid_detections}
        # Group previously known tracks by their class ID
        tracks_dict = {
            class_id: list(
                filter(lambda obj: obj.class_id == class_id, self._tracked_objects)
            )
            for class_id in tracked_classes
            if class_id in valid_detections_classes
        }

        # Remove detections too far from any track
        valid_detections = [
            det
            for det in valid_detections
            if self._has_track_candidate(det, tracks_dict[det.class_id])
        ]

        # Compute cost related to distance between detection and all of tracks within
        # allowed distance. Resulting list contains poses of detections with symmetries
        # already applied that minimize the difference cost.
        nested_minimal_symmetries = [
            self._find_minimum_costs(det, tracks_dict[det.class_id])
            for det in valid_detections
        ]
        # Fatten list of lists with list comprehension
        flat_minimal_symmetries = [
            ms for ms_arr in nested_minimal_symmetries for ms in ms_arr
        ]
        minimal_symmetries = sorted(
            flat_minimal_symmetries,
            key=lambda track: track.cost,
        )

        def _min_sym_cleaner(
            sym: TrackDetectionCloseness, track_id: str, idx: int
        ) -> bool:
            """Returns false if given symmetry has already found track_id or index.

            :param sym: Structure holding tracked symmetry.
            :type sym: TrackDetectionCloseness
            :param track_id: Name of track id expected to be filtered.
            :type track_id: str
            :param idx: Index of the detection expected to be filtered.
            :type idx: int
            :return: ``True`` if symmetry can be kep, ``False``
                if symmetry has to be filtered out.
            :rtype: bool
            """
            return sym.track_id != track_id and sym.idx != idx

        final_symmetries = []
        while len(minimal_symmetries) > 0:
            # First element in the list will always have minimal cost.
            best_detection = minimal_symmetries[0]
            final_symmetries.append(best_detection)
            # Remove all instances in the list containing already selected
            # track ID or index of the symmetry
            minimal_symmetries = list(
                filter(
                    lambda sym: _min_sym_cleaner(
                        sym, best_detection.track_id, best_detection.idx
                    ),
                    minimal_symmetries,
                )
            )

        # Assign previous track IDs to detected objects.
        for sym in final_symmetries:
            detections.detections[sym.idx].id = sym.track_id
            detections.detections[sym.idx].results[0].pose.pose = self._se3_to_pose(
                sym.transformation
            )

        # Find set of indices that did not have assigned tracks
        untracked_indices = set(range(len(detections.detections))) - {
            sym.idx for sym in final_symmetries
        }
        # Generate new tracks
        for i in untracked_indices:
            class_id = detections.detections[i].results[0].hypothesis.class_id
            if class_id not in self._object_track_counter:
                self._object_track_counter[class_id] = 0
            object_idx = self._object_track_counter[class_id]
            detections.detections[i].id = f"{class_id}_{object_idx}"
            self._object_track_counter[class_id] += 1

        return detections
