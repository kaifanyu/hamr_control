"""
Produce an optimized trajectory using wheel odometry,
IMU measurements, and RGB-D visual constraints.
"""
import bisect
import gtsam
from gtsam.symbol_shorthand import X, V, B
from rtabmap_msgs.msg import Info
from rtabmap_msgs.msg import MapData
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np


class gtsam_localization:
    def __init__(self):
        self.k = 0

        # Initialize IMU parameters
        self.prev_imu_time = None
        self.imu_bias = gtsam.imuBias.ConstantBias()
        self.imu_integrated = gtsam.PreintegratedImuMeasurements(
                    gtsam.PreintegrationParams.MakeSharedU(9.81),
                    self.imu_bias
                )
        self.bias_between_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([
                1e-3, 1e-3, 1e-3,  # accelerometer bias change
                1e-4, 1e-4, 1e-4   # gyroscope bias change
            ])
        )

        self.prev_odom_pose = None
        self.rgbd_noise = gtsam.noiseModel.Diagonal.Sigmas(
            [0.2, 0.2, 0.1, 0.1, 0.1, 0.1])
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            [0.2, 0.2, 0.1, 0.1, 0.1, 0.1])
        self.loop_noise = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.0),
            gtsam.noiseModel.Diagonal.Sigmas([0.2, 0.2, 0.1, 0.1, 0.1, 0.1])
        )

        # Timestamps of each GTSAM state X(k), in order of k
        self.odom_times = []
        # Maps RTAB-Map node id -> closest GTSAM k value
        self.rtab_to_gtsam = {}

        self.declare_parameter("odom_topic", "/wheel_odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("map_info_topic", "/rtabmap/info")
        self.declare_parameter("map_data_topic", "/rtabmap/mapData")

        # Initialize the factor graph
        self.graph = gtsam.NonlinearFactorGraph()

        # Add prior factors for the initial state to the graph
        initial_pose = gtsam.Pose3()
        initial_velocity = gtsam.Vector3(0, 0, 0)
        initial_bias = gtsam.imuBias.ConstantBias()
        pose_prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        velocity_prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            [0.1, 0.1, 0.1])
        self.bias_prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        self.graph.add(
            gtsam.PriorFactorPose3(X(0), initial_pose, pose_prior_noise))
        self.graph.add(
            gtsam.PriorFactorVector(V(0), initial_velocity, velocity_prior_noise))
        self.graph.add(
            gtsam.PriorFactorConstantBias(B(0), initial_bias, self.bias_prior_noise))

        # Initialize the initial values for the optimizer
        self.values = gtsam.Values()
        self.values.insert(X(0), initial_pose)
        self.values.insert(V(0), initial_velocity)
        self.values.insert(B(0), initial_bias)

        # Subscribe to RTAB map info topic
        self.map_graph_sub = self.create_subscription(
            Info,
            self.get_parameter("map_info_topic").value,
            self.map_info_callback,
            10
        )

        self.pending_closures = {}
        self.seen_closures = set()

        # Subscribe to RTAB map info topic
        self.map_data_sub = self.create_subscription(
            MapData,
            self.get_parameter("map_data_topic").value,
            self.map_data_callback,
            10
        )

        self.rtabmap_stamps = {}

        # Subscribe to wheel odometry topic
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            10
        )

        # Subscribe to IMU topic
        self.imu_sub = self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.imu_callback
        )

    def odom_callback(self, msg: Odometry):
        # Unpack message
        t = msg.pose.pose.position
        q = msg.pose.pose.orientation

        rotation = gtsam.Rot3.Quaternion(q.w, q.x, q.y, q.z)
        translation = gtsam.Point3(t.x, t.y, t.z) 
        odom_pose = gtsam.Pose3(rotation, translation)

        odom_time = (
            msg.header.stamp.sec +
            msg.header.stamp.nanosec * 1e-9
        )

        self.odom_times.append(odom_time)

        if self.prev_odom_pose is None:
            self.prev_odom_pose = odom_pose
            self.k += 1
            return

        # Add wheel odometry factor to graph
        odom_relative_pose = self.prev_odom_pose.between(odom_pose)

        self.graph.add(gtsam.BetweenFactorPose3(
            X(self.k - 1), X(self.k),
            odom_relative_pose,
            self.odom_noise
        ))

        # Add preintegrated IMU factor to graph if available
        if self.imu_integrated is not None:
            imu_factor = gtsam.ImuFactor(
                X(self.k - 1), V(self.k - 1),
                X(self.k), V(self.k),
                B(self.k - 1),
                self.imu_integrated
            )
            self.graph.add(imu_factor)

            # Reset preintegrated IMU measurements after adding factor
            self.imu_integrated.resetIntegrationAndSetBias(self.imu_bias)

        # Add bias factor to graph
        self.graph.add(gtsam.BetweenFactorConstantBias(
            B(self.k - 1), B(self.k),
            gtsam.imuBias.ConstantBias(),
            self.bias_between_noise
        ))

        # Guesses for new states
        self.values.insert(X(self.k), odom_pose)
        self.values.insert(V(self.k), gtsam.Vector3(0, 0, 0))
        self.values.insert(B(self.k), self.imu_bias)

        # Update previous odometry pose and increment step counter
        self.prev_odom_pose = odom_pose
        self.k += 1

    def _closest_gtsam_k(self, timestamp):
        if not self.odom_times:
            return None

        idx = bisect.bisect_left(self.odom_times, timestamp)
        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(self.odom_times):
            candidates.append(idx)

        best_idx = min(
            candidates, key=lambda i: abs(self.odom_times[i] - timestamp))
        return best_idx

    def _rtab_id_to_gtsam_k(self, node_id, timestamp):
        if node_id not in self.rtab_to_gtsam:
            self.rtab_to_gtsam[node_id] = self._closest_gtsam_k(timestamp)
        return self.rtab_to_gtsam[node_id]

    def imu_callback(self, msg: Imu):
        # Unpack IMU message
        a = msg.linear_acceleration
        w = msg.angular_velocity

        accel = gtsam.Vector3(a.x, a.y, a.z)
        gyro = gtsam.Vector3(w.x, w.y, w.z)

        current_time = (
            msg.header.stamp.sec +
            msg.header.stamp.nanosec * 1e-9
        )

        if self.prev_imu_time is None:
            self.prev_imu_time = current_time
            return

        # Integrate IMU measurements over the time interval
        dt = current_time - self.prev_imu_time
        self.prev_imu_time = current_time
        self.imu_integrated.integrateMeasurement(accel, gyro, dt)

    def map_data_callback(self, msg: MapData):
        for node in msg.nodes:
            self.rtabmap_stamps[node.id] = node.stamp

        self.process_pending_closures()

    def map_info_callback(self, msg: Info):
        if msg.loop_closure_id <= 0:
            return

        pair = (msg.ref_id, msg.loop_closure_id)

        if pair in self.seen_closures:
            return

        self.pending_closures[pair] = msg.loop_closure_transform

        self.process_pending_closures()

    def process_pending_closures(self):
        for pair, transform in list(self.pending_closures.items()):
            current_id, previous_id = pair

            if (current_id not in self.rtabmap_stamps or
                    previous_id not in self.rtabmap_stamps):
                continue

            # Get time and corresponding GTSAM keys for the current and
            # previous nodes.
            current_time = self.rtabmap_stamps[current_id]
            previous_time = self.rtabmap_stamps[previous_id]

            current_k = self._rtab_id_to_gtsam_k(
                current_id, current_time
            )
            previous_k = self._rtab_id_to_gtsam_k(
                previous_id, previous_time
            )

            if current_k is None or previous_k is None:
                continue

            # Convert the RTAB-Map transform into a GTSAM Pose3
            # and add it as a BetweenFactorPose3 to the graph.
            t = transform.translation
            q = transform.rotation
            rot = gtsam.Rot3.Quaternion(q.w, q.x, q.y, q.z)
            trans = gtsam.Point3(t.x, t.y, t.z)
            relative_pose = gtsam.Pose3(rot, trans)

            self.graph.add(
                gtsam.BetweenFactorPose3(
                    X(current_k),
                    X(previous_k),
                    relative_pose,
                    self.loop_noise
                )
            )

            self.seen_closures.add(pair)
            del self.pending_closures[pair]
