import rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
import numpy as np
import math
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import cv2
from copy import deepcopy
from scipy.interpolate import griddata
import torch
import torch.nn as nn
import torchvision.transforms.functional as F
import torch.nn.functional as F_torch
import torch.nn.parallel as parallel
from multiprocessing import Pool
import sys
import os
from wait_for_it.utils import to_grid_msg

# Add parent directory to path to import verti-arena-map (with hyphen in directory name)
_src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

from wait_for_it.utils import *
import matplotlib.pyplot as plt

from wait_for_it.utils import quaternion_to_yaw

from typing import Tuple, Optional

class StaticMap:
    """Load and query patches from a fused static map of the arena."""
    def __init__(self, data_path: str, meta_path: str):
        if not os.path.exists(data_path):
            raise FileNotFoundError(f"Data file not found: {data_path}")
        if not os.path.exists(meta_path):
            raise FileNotFoundError(f"Metadata file not found: {meta_path}")

        # Load metadata
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.width = None
        self.height = None
        self.layers = None
        self._load_metadata(meta_path)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # Load map layers
        self.maps = {}
        layers = self.layers if self.layers else ["elevation"]
        data_dir = os.path.dirname(os.path.abspath(data_path))

        for layer in layers:
            layer_file = os.path.join(data_dir, f"fused_{layer}_map.npy")
            if not os.path.exists(layer_file):
                # Fallback for elevation data file
                if layer == "elevation" and os.path.exists(data_path):
                    layer_file = data_path
                else:
                    continue

            grid = np.load(layer_file).astype(np.float32, copy=False)
            self.maps[layer] = torch.from_numpy(grid).to(device=self.device).float()

        if not self.maps:
            raise FileNotFoundError("No layer maps found to load")

        # Set default/primary map reference
        first_layer = "elevation" if "elevation" in self.maps else next(iter(self.maps.keys()))
        self.map = self.maps[first_layer]
        self.height_px, self.width_px = self.map.shape

    def _load_metadata(self, meta_path: str) -> None:
        with open(meta_path, "r", encoding="ascii") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 2:
                    continue
                key, val = parts
                if key == "resolution":
                    self.resolution = float(val)
                elif key == "origin_x":
                    self.origin_x = float(val)
                elif key == "origin_y":
                    self.origin_y = float(val)
                elif key == "width":
                    self.width = int(val)
                elif key == "height":
                    self.height = int(val)
                elif key == "layers":
                    self.layers = val.split(",")

        if self.resolution is None or self.origin_x is None or self.origin_y is None:
            raise RuntimeError("Missing required metadata fields")

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def get_patch(self, x: float, y: float, patch_size: int, layer: str = "elevation") -> torch.Tensor:
        gx, gy = self.world_to_grid(x, y)
        half = patch_size // 2
        x1 = gx - half
        x2 = gx + half + (patch_size % 2)
        y1 = gy - half
        y2 = gy + half + (patch_size % 2)

        map_t = self.maps.get(layer, self.map)
        if map_t.ndim == 3 and map_t.shape[2] == 3:
            patch_shape = (patch_size, patch_size, 3)
        else:
            patch_shape = (patch_size, patch_size)

        # Create patch with NaN padding
        patch = torch.full(patch_shape, float('nan'), dtype=torch.float32, device=self.device)

        # Calculate valid region
        src_x1 = max(0, x1)
        src_x2 = min(self.width_px, x2)
        src_y1 = max(0, y1)
        src_y2 = min(self.height_px, y2)

        dst_x1 = src_x1 - x1
        dst_x2 = dst_x1 + (src_x2 - src_x1)
        dst_y1 = src_y1 - y1
        dst_y2 = dst_y1 + (src_y2 - src_y1)

        # Copy valid region
        patch[dst_y1:dst_y2, dst_x1:dst_x2] = map_t[src_y1:src_y2, src_x1:src_x2]
        return patch

class MapProcessor:
    def __init__(self, elevation_map: StaticMap = None, wheelbase=0.312, 
                 track=0.2, wheel_patch=0.05, use_smoothing=True, smoothing_sigma=1.0,
                 ros_publish=True):
        self.map = GridMap()
        self.map_elevation = np.zeros((600, 600), dtype=np.float32)
        self.map_traversal = np.zeros((600, 600), dtype=np.float32)
        
        # Reference to full elevation map for fallback queries (auto-load if None)
        if elevation_map is None:
            static_maps_dir = os.path.join(_src_dir, 'static_maps')
            data_path = os.path.join(static_maps_dir, 'fused_elevation_map.npy')
            meta_path = os.path.join(static_maps_dir, 'fused_elevation_map_meta.txt')
            try:
                self.elevation_map = StaticMap(data_path=data_path, meta_path=meta_path)
            except Exception as e:
                rospy.logwarn(f"Failed to load static elevation map: {e}")
                self.elevation_map = None
        else:
            self.elevation_map = elevation_map

        self.map_elev_img = np.zeros((600, 600), dtype=np.uint8)
        self.map_trav_img = np.zeros((600, 600), dtype=np.uint8)
        self.map_semantic = None
        self.semantic_layer_name = None

        self.map_pose = GridMap().info.pose
        self.map_length = GridMap().info.length_x
        self.map_width = GridMap().info.length_y
        self.map_res = GridMap().info.resolution
        self.map_layers = GridMap().layers
        self.map_shape = [250,250]
        self.map_init = False
        # choose device automatically to avoid errors when CUDA is not present
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.robot_pose = Pose()
        self.map_embedding = None
        
        self.fill = 6
        self.fill_val_div = 2

        # Default missing-value fallbacks (user-provided)
        # Elevation default in meters
        self.default_elevation = float(-0.03323321416974068)
        # Semantic (dino) default RGB floats
        self.default_semantic = torch.tensor([0.6321726, 0.23620692, 0.8820262], dtype=torch.float32, device=self.device)
        
        # DINOv3 PCA feature layers - can have different resolution than elevation
        self.map_dino_0 = None
        self.map_dino_1 = None
        self.map_dino_2 = None
        self.dino_shape = None  # Will be set when DINO data is ingested
        self.dino_res = None    # Resolution of DINO layers
        self.dino_pose = None   # Pose/origin of DINO map (may differ from elevation)

        # ROS publishing options for debug
        self.ros_publish = ros_publish
        self.processed_topic = "/processed_gridmap"
        if self.ros_publish:
            try:
                self.pub_processed = rospy.Publisher(self.processed_topic, GridMap, queue_size=1, latch=True)
            except Exception:
                self.pub_processed = None

        # vehicle geometry (meters)
        self.set_vehicle_dimensions(wheelbase, track, wheel_patch)
        
        # smoothing parameters for noise reduction
        self.use_smoothing = use_smoothing
        self.smoothing_sigma = smoothing_sigma  # Gaussian kernel sigma in pixels

    def pack_rgb_torch(self, rgb_t, normalize=True):
        """Pack an RGB tensor of shape (H, W, 3) with values in [0, 1] into a (H, W) packed float32 grid."""
        # Find which pixels are NaN
        nan_mask = ~torch.isfinite(rgb_t).all(dim=2)
        
        # Replace NaNs with 0 temporarily to avoid casting undefined integers
        rgb_safe = torch.where(torch.isnan(rgb_t), torch.tensor(0.0, device=rgb_t.device), rgb_t)
        
        if normalize:
            rgb_clamped = torch.clamp(rgb_safe * 255.0, 0.0, 255.0)
        else:
            rgb_clamped = torch.clamp(rgb_safe, 0.0, 255.0)
        r = rgb_clamped[:, :, 0].to(torch.int32)
        g = rgb_clamped[:, :, 1].to(torch.int32)
        b = rgb_clamped[:, :, 2].to(torch.int32)
        packed_int = (r << 16) | (g << 8) | b
        packed_float = packed_int.view(torch.float32)
        
        # Restore NaNs in the packed float representation
        packed_float = torch.where(nan_mask, torch.tensor(float('nan'), device=rgb_t.device), packed_float)
        return packed_float

    def _extract_pose_components(self, msg):
        """Return (x,y,z,yaw) from a Pose, PoseStamped-like, or array-like input using duck typing."""
        # PoseStamped-like: has .pose
        if hasattr(msg, 'pose'):
            p = msg.pose
            if hasattr(p, 'position') and hasattr(p, 'orientation'):
                x = float(p.position.x)
                y = float(p.position.y)
                z = float(p.position.z)
                yaw = quaternion_to_yaw(p.orientation)
                return x, y, z, yaw

        # Pose-like: has position/orientation
        if hasattr(msg, 'position') and hasattr(msg, 'orientation'):
            x = float(msg.position.x)
            y = float(msg.position.y)
            z = float(msg.position.z)
            yaw = quaternion_to_yaw(msg.orientation)
            return x, y, z, yaw

        # indexable (list/tuple/np.array): [x,y,z,..., yaw at idx 5 optional]
        try:
            x = float(msg[0]); y = float(msg[1])
        except Exception:
            raise ValueError('pose must be Pose-like or array-like')
        z = float(msg[2]) if len(msg) > 2 else float(self.robot_pose.position.z)
        yaw = float(msg[5]) if len(msg) > 5 else quaternion_to_yaw(self.robot_pose.orientation)
        return x, y, z, yaw

    def _extract_xy(self, msg):
        """Return (x,y) from pose-like or array-like input."""
        if hasattr(msg, 'pose'):
            p = msg.pose
            return float(p.position.x), float(p.position.y)
        if hasattr(msg, 'position'):
            return float(msg.position.x), float(msg.position.y)
        try:
            return float(msg[0]), float(msg[1])
        except Exception:
            raise ValueError('pose must be Pose-like or array-like')
        
    #update map data
    def update_map(self, map_data):
        self.map = map_data
        self.update_attributes()
        
        # Process and fill gaps for all layers in self.map in-place
        for i, layer_name in enumerate(self.map.layers):
            processed_grid = self.process_layer(self.map.data[i].data, layer_name=layer_name)
            
            # Keep elevation layer as attribute for backward compatibility
            if layer_name == 'elevation':
                self.map_elevation = processed_grid
                
            # Convert back to flat list for the ROS message
            if isinstance(processed_grid, torch.Tensor):
                processed_np = processed_grid.detach().cpu().numpy()
            else:
                processed_np = np.array(processed_grid, dtype=np.float32)
                
            try:
                processed_np = np.flip(processed_np, axis=0)
                processed_np = np.flip(processed_np, axis=1)
            except Exception:
                pass
                
            self.map.data[i].data = processed_np.flatten().astype(np.float32).tolist()

        # Decode and process semantic layer if present
        semantic_idx = None
        semantic_layer_name = None
        for i, layer_name in enumerate(self.map.layers):
            if layer_name == 'dino':
                semantic_idx = i
                semantic_layer_name = layer_name
                break
        if semantic_idx is not None:
            r, g, b = self._decode_color_layer(self.map.data[semantic_idx].data, self.map_shape)
            semantic_rgb = torch.stack([r, g, b], dim=-1)
            self.map_semantic = self.process_color_layer(semantic_rgb)
            self.semantic_layer_name = semantic_layer_name
        else:
            semantic_rgb = torch.zeros((*self.map_shape, 3), dtype=torch.float32, device=self.device)
            self.map_semantic = self.process_color_layer(semantic_rgb)
            self.semantic_layer_name = 'dino'
        
        # Update DINO layers if present in the GridMap
        self.update_dino_layers(self.map, self.map_semantic)
        
        # Apply Gaussian smoothing to reduce noise (improves roll/pitch estimation)
        if self.use_smoothing and self.smoothing_sigma > 0:
            self.map_elevation = F.gaussian_blur(
                self.map_elevation.unsqueeze(0), 
                [5, 5],  # kernel size computed from sigma
                [self.smoothing_sigma]
            ).squeeze()

        self.map_init = True

        # Cache the elevation map as a (1,1,H,W) tensor for efficient grid_sample calls
        if isinstance(self.map_elevation, torch.Tensor):
            self._map_single = self.map_elevation.unsqueeze(0).unsqueeze(0).to(self.device)
        else:
            self._map_single = torch.from_numpy(self.map_elevation.astype(np.float32)).unsqueeze(0).unsqueeze(0).to(self.device)

        # Optionally publish the processed gridmap for debugging
        if getattr(self, 'ros_publish', False) and getattr(self, 'pub_processed', None) is not None:
            try:
                # Prepare a copy of the processed GridMap
                out = deepcopy(self.map)

                # If the semantic layer was computed but not in the GridMap layers, append it
                if self.semantic_layer_name not in out.layers and self.map_semantic is not None:
                    sem = self.map_semantic
                    if isinstance(sem, torch.Tensor):
                        sem_np = sem.detach().cpu().numpy()
                    else:
                        sem_np = np.array(sem, dtype=np.float32)

                    try:
                        sem_np = np.flip(sem_np, axis=0)
                        sem_np = np.flip(sem_np, axis=1)
                    except Exception:
                        pass

                    sem_name = self.semantic_layer_name or "semantic"
                    sem_layer = Float32MultiArray()
                    sem_layer.data = self._encode_color_layer(sem_np).flatten().astype(np.float32).tolist()

                    shape = sem_np.shape
                    dim1 = MultiArrayDimension()
                    dim1.label = "column_index"
                    dim1.size = shape[0]
                    dim1.stride = shape[0] * shape[1]

                    dim2 = MultiArrayDimension()
                    dim2.label = "row_index"
                    dim2.size = shape[1]
                    dim2.stride = shape[1]

                    sem_layer.layout.data_offset = 0
                    sem_layer.layout.dim.append(dim1)
                    sem_layer.layout.dim.append(dim2)

                    out.layers.append(sem_name)
                    out.data.append(sem_layer)
                
                if self.pub_processed.get_num_connections():
                    self.pub_processed.publish(out)

            except Exception as e:
                rospy.logdebug(f'MapProcessor: failed to publish processed gridmap: {e}')
    
    def update_dino_layers(self, map_data, semantic_map=None):
        """Extract and store DINOv3 PCA feature layers from GridMap message.
        
        Looks for layers named 'dino_0', 'dino_1', and 'dino_2' in the GridMap.
        These layers may have different resolution than the elevation data.
        
        Args:
            map_data: GridMap message potentially containing DINO layers
        """
        # Find DINO layer indices
        dino_indices = {}
        color_idx = None
        self.map_data = map_data
        for i, layer_name in enumerate(map_data.layers):
            if layer_name == 'dino':
                color_idx = i
        
        # # If no DINO or color layers found, nothing to update
        # if not dino_indices and color_idx is None:
        #     return
        
        # Store map metadata (may differ from elevation map)
        self.dino_pose = map_data.info.pose
        self.dino_res = map_data.info.resolution
        dino_length = map_data.info.length_x
        dino_width = map_data.info.length_y
        self.dino_shape = (
            int(dino_length // self.dino_res), 
            int(dino_width // self.dino_res)
        )

        # Prefer explicit color layer if present (packed 0xRRGGBB in float bits)
        if semantic_map is not None and semantic_map.shape[2] == 3:
            self.map_dino_0 = semantic_map[:, :, 0]
            self.map_dino_1 = semantic_map[:, :, 1]
            self.map_dino_2 = semantic_map[:, :, 2]
            rospy.logdebug("Updated DINO layers from separate semantic RGB layer")
        
        elif color_idx is not None:
            r, g, b = self._decode_color_layer(map_data.data[color_idx].data, self.dino_shape)
            self.map_dino_0 = r
            self.map_dino_1 = g
            self.map_dino_2 = b
            rospy.logdebug("Updated DINO layers from packed color channel")
            return
                
    def _process_dino_layer(self, layer_data, shape):
        """Process a single DINO feature layer.
        
        Args:
            layer_data: Flat list/array of feature values
            shape: Tuple (height, width) for reshaping
            
        Returns:
            Processed layer as torch tensor on device
        """
        # Convert to torch tensor
        # arr = np.array(layer_data, dtype=np.float32)
        layer_t = torch.from_numpy(np.array(layer_data, dtype=np.float32)).to(self.device)
        
        # Handle NaN values - for DINO features, use 0 as default
        valid_mask = torch.isfinite(layer_t)
        if valid_mask.sum() == 0:
            # All NaN -> return zeros
            layer_t[:] = 0.0
        else:
            # Replace NaN with median of valid values (or 0 if you prefer)
            median_val = torch.median(layer_t[valid_mask])
            layer_t[~valid_mask] = 0.0#median_val
        
        # Reshape and flip both axes to match elevation map orientation
        layer_grid = layer_t.reshape(tuple(shape))
        layer_grid = torch.flip(layer_grid, dims=(0, 1))

        # Min Max normalization 
        valid_mask_2d = torch.isfinite(layer_grid)
        if valid_mask_2d.sum() > 0:
            valid_vals = layer_grid[valid_mask_2d]
            min_val = torch.min(valid_vals)
            max_val = torch.max(valid_vals)
            if max_val > min_val:  # Avoid division by zero
                layer_grid[valid_mask_2d] = (valid_vals - min_val) / (max_val - min_val)
            else:
                layer_grid[valid_mask_2d] = 0.0  # or 1.0, since all values are the same
        
        return layer_grid

    def _decode_color_layer(self, layer_data, shape, normalize=True):
        """Unpack a grid_map color layer stored as float bit-pattern 0xRRGGBB.

        Args:
            layer_data: flat iterable of float32 values whose bits encode the color
            shape: (h, w) target shape
            normalize: divide channels by 255 if True

        Returns:
            Tuple of torch tensors (r, g, b) shaped as `shape` on self.device
        """
        arr = np.array(layer_data, dtype=np.float32)
        color_uint = arr.view(np.uint32)

        def make_channel(bits):
            ch = torch.from_numpy(bits.astype(np.float32)).to(self.device)
            ch = ch.reshape(tuple(shape))
            ch = torch.flip(ch, dims=(0, 1))
            return ch / 255.0 if normalize else ch

        r = make_channel((color_uint >> 16) & 0xFF)
        g = make_channel((color_uint >> 8) & 0xFF)
        b = make_channel(color_uint & 0xFF)
        return r, g, b

    def _encode_color_layer(self, rgb, normalize=True):
        """Pack an RGB layer into grid_map color float32 bit-pattern 0xRRGGBB."""
        arr = np.asarray(rgb)
        if arr.ndim != 3 or arr.shape[2] != 3:
            raise ValueError('semantic layer must be shape (H, W, 3)')

        if normalize:
            arr = np.clip(arr * 255.0, 0.0, 255.0)
        else:
            arr = np.clip(arr, 0.0, 255.0)

        r = arr[:, :, 0].astype(np.uint32)
        g = arr[:, :, 1].astype(np.uint32)
        b = arr[:, :, 2].astype(np.uint32)
        packed = (r << 16) | (g << 8) | b
        return packed.view(np.float32)

    #process layer data to remove NaN and reshape
    def process_layer(self, layer_data, layer_name="elevation"):
        # Prefer tensor-based processing for performance and compatibility
        arr = np.array(layer_data, dtype=np.float32)
        layer_t = torch.from_numpy(arr).to(self.device)

        if layer_name in ('rgb', 'dino'):
            # For color layers, both non-finite values and exact 0.0 values represent missing/unobserved cells
            valid_mask = torch.isfinite(layer_t) & (layer_t != 0.0)
        else:
            valid_mask = torch.isfinite(layer_t)

        # All-NaN/invalid guard -> return zeros in expected shape
        if valid_mask.numel() == 0 or valid_mask.sum() == 0:
            # No valid samples: fill entire grid with default value
            if layer_name in ('rgb', 'dino'):
                default_rgb = self.default_semantic.view(1, 1, 3)
                packed_default = self.pack_rgb_torch(default_rgb).item()
                layer_t[:] = float(packed_default)
            else:
                layer_t[:] = float(self.default_elevation)
            layer_grid = layer_t.reshape(tuple(self.map_shape))
            layer_grid = torch.flip(layer_grid, dims=(0, 1))
            return layer_grid

        valid_vals = layer_t[valid_mask]
        min_value = torch.min(valid_vals)
        max_value = torch.max(valid_vals)

        # Fill strategies using tensor ops
        if self.fill == 0:
            median = torch.median(valid_vals)
            layer_t[~valid_mask] = median
        elif self.fill == 1:
            layer_t[~valid_mask] = max_value
        elif self.fill == 2:
            layer_t[~valid_mask] = min_value
        elif self.fill == 3:
            layer_t[~valid_mask] = (max_value - min_value) / float(self.fill_val_div)
        elif self.fill == 4:
            layer_t[~valid_mask] = float(self.robot_pose.position.z) - 0.152
        elif self.fill == 5:
            # keep NaNs
            pass

        # reshape and flip both axes in a single call
        layer_grid = layer_t.reshape(tuple(self.map_shape))
        layer_grid = torch.flip(layer_grid, dims=(0, 1))
        
        #Handle fill==6 after flipping so index->world mapping matches displayed map
        if self.fill == 6:
            if layer_name in ('rgb', 'dino'):
                nan_mask2d = ~torch.isfinite(layer_grid) | (layer_grid == 0.0)
            else:
                nan_mask2d = ~torch.isfinite(layer_grid)

            # Try to fill from elevation_map patch if available
            try:
                patch = None
                if self.elevation_map is not None and hasattr(self.elevation_map, 'get_patch'):
                    patch = self.elevation_map.get_patch(
                        self.map_pose.position.x, 
                        self.map_pose.position.y, 
                        self.map_shape[0],
                        layer=layer_name
                    )
                if patch is not None:
                    # patch may be numpy or torch; coerce and apply where finite
                    if not isinstance(patch, torch.Tensor):
                        patch_t = torch.from_numpy(np.array(patch, dtype=np.float32)).to(self.device)
                    else:
                        patch_t = patch.to(self.device)
                    
                    if patch_t.ndim == 3 and patch_t.shape[2] == 3:
                        patch_t = self.pack_rgb_torch(patch_t)
                        
                    layer_grid[nan_mask2d] = patch_t[nan_mask2d]
            except Exception:
                pass
            # Any remaining NaNs -> fill with configured default
            if layer_name in ('rgb', 'dino'):
                nan_mask2d = ~torch.isfinite(layer_grid) | (layer_grid == 0.0)
            else:
                nan_mask2d = ~torch.isfinite(layer_grid)

            if nan_mask2d.any():
                if layer_name in ('rgb', 'dino'):
                    default_rgb = self.default_semantic.view(1, 1, 3)
                    packed_default = self.pack_rgb_torch(default_rgb).item()
                    fill_val = torch.tensor(float(packed_default), dtype=layer_grid.dtype, device=self.device)
                else:
                    fill_val = torch.tensor(float(self.default_elevation), dtype=layer_grid.dtype, device=self.device)
                layer_grid[nan_mask2d] = fill_val
        return layer_grid

    def process_color_layer(self, rgb_t):
        """Process a packed color layer using the same fill rules as elevation."""
        if not isinstance(rgb_t, torch.Tensor):
            rgb_t = torch.from_numpy(np.array(rgb_t, dtype=np.float32)).to(self.device)
        else:
            rgb_t = rgb_t.to(self.device)

        if rgb_t.ndim != 3 or rgb_t.shape[2] != 3:
            raise ValueError('semantic layer must be shape (H, W, 3)')

        # Consider pixels with all channels equal to 0.0 as missing
        zero_mask = torch.isclose(rgb_t, torch.tensor(0.0, dtype=rgb_t.dtype, device=self.device))
        valid_mask = ~zero_mask.all(dim=2)
        # if valid_mask.numel() == 0 or valid_mask.sum() == 0:
        #     # No valid semantic samples: fill entire semantic map with configured default color
        #     default_rgb = self.default_semantic.view(1, 1, 3).to(device=self.device)
        #     return torch.ones_like(rgb_t) * default_rgb

        # # If some semantic values are missing and fill mode 6 is enabled,
        # # pre-fill the missing pixels with the configured default color so
        # # later fill==6 logic can use this as a base (and optionally overwrite
        # # from an elevation/dino patch). This avoids leaving NaN as the base.
        # if valid_mask.numel() != 0 and valid_mask.sum() < valid_mask.numel() and self.fill == 6:
        #     default_rgb = self.default_semantic.view(1, 1, 3).to(device=self.device)
        #     rgb_t[~valid_mask] = default_rgb
        #     # Recompute mask so subsequent min/max computations see filled values
        #     zero_mask = torch.isclose(rgb_t, torch.tensor(0.0, dtype=rgb_t.dtype, device=self.device))
        #     valid_mask = ~zero_mask.all(dim=2)
        if valid_mask.numel() == 0 or valid_mask.sum() == 0:
            # No valid values in any channel, so fill with default color
            self.fill = 6

        if self.fill < 6:
            for c in range(3):
                channel = rgb_t[:, :, c]
                valid_vals = channel[valid_mask]
                min_value = torch.min(valid_vals)
                max_value = torch.max(valid_vals)

                if self.fill == 0:
                    median = torch.median(valid_vals)
                    channel[~valid_mask] = median
                elif self.fill == 1:
                    channel[~valid_mask] = max_value
                elif self.fill == 2:
                    channel[~valid_mask] = min_value
                elif self.fill == 3:
                    channel[~valid_mask] = (max_value - min_value) / float(self.fill_val_div)
                elif self.fill == 4:
                    channel[~valid_mask] = float(self.robot_pose.position.z) - 0.152
                elif self.fill == 5:
                    pass

                rgb_t[:, :, c] = channel

        elif self.fill == 6:
            # Missing semantic pixels are those that are all-zero across channels
            nan_mask2d = ~(~torch.isclose(rgb_t, torch.tensor(0.0, dtype=rgb_t.dtype, device=self.device)).all(dim=2))
            if self.elevation_map is not None and hasattr(self.elevation_map, 'maps') and 'dino' in self.elevation_map.maps:
                patch = self.elevation_map.get_patch(
                    self.map_pose.position.x,
                    self.map_pose.position.y,
                    self.map_shape[0],
                    layer='dino'
                )
                try:
                    if isinstance(patch, torch.Tensor):
                        patch = patch.to(self.device)
                    else:
                        patch = torch.from_numpy(np.array(patch, dtype=np.float32)).to(self.device)
                    rgb_t[:] = torch.maximum(rgb_t, patch)
                except Exception:
                    pass

            # Any remaining NaNs -> fill with configured default semantic color
            nan_mask2d = ~torch.isfinite(rgb_t).all(dim=2)
            if nan_mask2d.any():
                fill_rgb = self.default_semantic.to(device=self.device, dtype=rgb_t.dtype)
                rgb_t[nan_mask2d] = fill_rgb

        return rgb_t

    #update map info data
    def update_attributes(self):
        self.map_pose = self.map.info.pose
        self.map_length = self.map.info.length_x
        self.map_width = self.map.info.length_y
        self.map_res = self.map.info.resolution
        self.map_layers = self.map.layers
        self.map_shape = (int(self.map_length//self.map_res), int(self.map_width//self.map_res))

    def set_vehicle_dimensions(self, wheelbase, track, wheel_patch):
        """Set vehicle geometry in meters.

        wheelbase: distance between front and rear axles (m)
        track: distance between left and right wheels (m)
        wheel_patch: tuple(length, width) of wheel contact patch (m)
        """
        self.wheelbase = float(wheelbase)
        self.track = float(track)
        self.robot_footprint = (float(wheelbase), float(track))
        # accept scalar or tuple (use first element as patch size)
        if hasattr(wheel_patch, '__iter__'):
            self.wheel_patch = float(wheel_patch[0])
        else:
            self.wheel_patch = float(wheel_patch)

    def set_smoothing(self, enabled=True, sigma=1.0):
        """Configure elevation map smoothing to reduce noise.
        
        enabled: whether to apply Gaussian smoothing
        sigma: smoothing strength in pixels (higher = more smoothing)
               Recommended range: 0.5 - 2.0
        """
        self.use_smoothing = bool(enabled)
        self.smoothing_sigma = float(sigma)

    def estimate_roll_pitch(self, pose=None):
        """Estimate roll and pitch based on the elevation gridmap.

        Method: sample the elevation at the four wheel contact centers
        (front-left, front-right, rear-left, rear-right) and compute
        roll = atan2((right_avg - left_avg), track)
        pitch = atan2((front_avg - rear_avg), wheelbase)

        pose can be a ROS `Pose` message or an array-like. If None, uses
        `self.robot_pose`.

        Returns a dict with radians and degrees and sampled wheel heights.
        """
        # extract pose components using duck typing
        if pose is None:
            px, py, pz, yaw = self._extract_pose_components(self.robot_pose)
        else:
            px, py, pz, yaw = self._extract_pose_components(pose)
        

        if self.wheelbase == 0.0 or self.track == 0.0:
            return {
                'roll_rad': None,
                'pitch_rad': None,
                'roll_deg': None,
                'pitch_deg': None,
                'wheel_heights': None,
                'error': 'wheelbase and track must be set via set_vehicle_dimensions()'
            }

        # wheel offsets in robot frame (x forward, y left).
        # The elevation map is already provided in robot yaw orientation, so
        # wheel offsets do not need to be rotated by the robot yaw — apply
        # offsets directly in the map/world coordinates used by `get_pose_height`.
        half_wb = self.wheelbase / 2.0
        half_tr = self.track / 2.0

        offsets = {
            'fl': ( half_wb,  half_tr),
            'fr': ( half_wb, -half_tr),
            'rl': (-half_wb,  half_tr),
            'rr': (-half_wb, -half_tr),
        }


        heights = {}
        r_pose = torch.zeros((1, 6), dtype=torch.float32)
        r_pose[0, 0] = px
        r_pose[0, 1] = py
        r_pose[0, 5] = yaw
        r_wheel = torch.zeros((1, 6), dtype=torch.float32)

        for key, (dx, dy) in offsets.items():
            # map is not in robot yaw frame need to get the world coordinates of each wheel
            r_wheel[0, 0] = dx
            r_wheel[0, 1] = dy

            w_wheels = to_world_torch(r_pose, r_wheel).squeeze().numpy()

            wx = w_wheels[0]
            wy = w_wheels[1]
            # query elevation at this map location; provide a guess z
            h = self.get_patch_height([wx, wy, pz], self.wheel_patch)
            try:
                heights[key] = float(h)
            except Exception:
                heights[key] = None

        # If any height is None, we can still try to compute with available values
        fl = heights.get('fl'); fr = heights.get('fr'); rl = heights.get('rl'); rr = heights.get('rr')

        if None in (fl, fr, rl, rr):
            return {
                'roll_rad': None,
                'pitch_rad': None,
                'roll_deg': None,
                'pitch_deg': None,
                'wheel_heights': heights,
                'error': 'one or more wheel heights unavailable'
            }

        left_avg = (fl + rl) / 2.0
        right_avg = (fr + rr) / 2.0
        front_avg = (fl + fr) / 2.0
        rear_avg = (rl + rr) / 2.0

        # wheel-sample estimates (kept for fallback and debugging)
        roll = math.atan2((left_avg - right_avg), self.track)
        # TF pitch convention (yaw-pitch-roll) is nose-down positive; flip to match quaternion_to_pitch/ground truth
        pitch = -math.atan2((front_avg - rear_avg), self.wheelbase)

        # Prefer a least-squares plane fit over the footprint for roll/pitch
        # If the plane fit fails or reports an error, fall back to wheel-sampled atan2 values.
        # try:
        #     plane = self.fit_plane_to_footprint(pose=[px, py, pz, 0, 0, yaw], return_points=False)
        #     if isinstance(plane, dict) and 'error' in plane:
        #         # plane fit reported an error; fallback
        #         roll = wheel_roll
        #         pitch = wheel_pitch
        #     else:
        #         # use plane-derived slopes (roll/pitch stored in radians)
        #         roll = float(plane.get('roll_rad', wheel_roll))
        #         pitch = float(plane.get('pitch_rad', wheel_pitch))
        # except Exception:
        #     plane = {'error': 'plane fit failed'}
        #     roll = wheel_roll
        #     pitch = wheel_pitch

        return {
            'roll_rad': roll,
            'pitch_rad': pitch,
            'roll_deg': math.degrees(roll),
            'pitch_deg': math.degrees(pitch),
            'wheel_heights': heights,
            # 'plane_fit': plane,
            'pose_used': {'x': px, 'y': py, 'z': pz, 'yaw': yaw},
            'wheel_sampled_height': {'fl': fl, 'fr': fr, 'rl': rl, 'rr': rr}
        }
    
    def estimate_roll_pitch_batch(self, poses, z_guess=None, return_torch=True):
        """Estimate roll and pitch for a batch of poses using torch for transforms.

        poses: numpy array or torch tensor of shape (N,3) or (N,4) or (N,6). Expected columns:
            [x, y, yaw] or [x, y, z, yaw] or full pose-like rows. If z is missing a
            fallback `z_guess` or `self.robot_pose.position.z` is used.

        z_guess: optional scalar z to use when poses do not contain z.
        return_torch: if True, return torch tensors instead of numpy arrays.

        Returns:
            results: torch tensor or numpy array shape (N,6) with columns [x, y, z, roll_rad, pitch_rad, yaw]
        """
        device = self.device
        
        # Accept both numpy and torch, normalize to torch tensor on device
        if isinstance(poses, torch.Tensor):
            poses_t = poses.to(device)
        else:
            poses_t = torch.from_numpy(np.array(poses, dtype=np.float32)).to(device)
        
        if poses_t.ndim != 2 or poses_t.shape[1] < 3:
            raise ValueError('poses must be shape (N,3+) with at least x,y,yaw')

        N = poses_t.shape[0]
        
        t_xs = poses_t[:, 0]
        t_ys = poses_t[:, 1]
        
        if poses_t.shape[1] >= 6:
            t_zs = poses_t[:, 2]
            t_yaws = poses_t[:, 5]
        elif poses_t.shape[1] == 4:
            t_zs = poses_t[:, 2]
            t_yaws = poses_t[:, 3]
        elif poses_t.shape[1] == 3:
            z_val = float(z_guess if z_guess is not None else self.robot_pose.position.z)
            t_zs = torch.full((N,), z_val, dtype=torch.float32, device=device)
            t_yaws = poses_t[:, 2]
        else:
            z_val = float(z_guess if z_guess is not None else self.robot_pose.position.z)
            t_zs = torch.full((N,), z_val, dtype=torch.float32, device=device)
            t_yaws = torch.zeros((N,), dtype=torch.float32, device=device)

        # wheel offsets (dx forward, dy left) order: fl, fr, rl, rr
        half_wb = self.wheelbase / 2.0
        half_tr = self.track / 2.0
        offsets = torch.tensor([[ half_wb,  half_tr],
                                [ half_wb, -half_tr],
                                [-half_wb,  half_tr],
                                [-half_wb, -half_tr]], dtype=torch.float32, device=device)  # (4,2)

        # Vectorized yaw rotation of all 4 wheel offsets at once (replaces loop + to_world_torch)
        cos_yaw = torch.cos(t_yaws)  # (N,)
        sin_yaw = torch.sin(t_yaws)  # (N,)
        # offsets[:, 0] = dx, offsets[:, 1] = dy -> (4,)
        # Rotate each offset by each yaw: wx = x + cos*dx - sin*dy, wy = y + sin*dx + cos*dy
        # Result shapes: (N, 4)
        off_dx = offsets[:, 0].unsqueeze(0)  # (1, 4)
        off_dy = offsets[:, 1].unsqueeze(0)  # (1, 4)
        wx = t_xs.unsqueeze(1) + cos_yaw.unsqueeze(1) * off_dx - sin_yaw.unsqueeze(1) * off_dy  # (N, 4)
        wy = t_ys.unsqueeze(1) + sin_yaw.unsqueeze(1) * off_dx + cos_yaw.unsqueeze(1) * off_dy  # (N, 4)

        center = (int(self.map_shape[0]//2), int(self.map_shape[1]//2))
        origin_x = float(self.map_pose.position.x)
        origin_y = float(self.map_pose.position.y)
        res = float(self.map_res)
        H = int(self.map_shape[0])
        W = int(self.map_shape[1])

        # patch size in pixels for wheel patch averaging (>=1)
        patch_px = max(1, int(round(self.wheel_patch / res))) if self.wheel_patch else 1

        # Use cached elevation map tensor (1,1,H,W) on device
        map_single = self._map_single if hasattr(self, '_map_single') else None
        if map_single is None:
            if not (isinstance(self.map_elevation, torch.Tensor)):
                map_t = torch.from_numpy(self.map_elevation.astype(np.float32))
            else:
                map_t = self.map_elevation 
            map_single = map_t.unsqueeze(0).unsqueeze(0).to(device)  # (1,1,H,W)

        # pixel coordinates (floating) for wheel centers
        center_col = float(center[1])
        center_row = float(center[0])
        pix_x = center_col + (wx - origin_x) / res + 1.0  # (N,4)
        pix_y = center_row + (wy - origin_y) / res + 1.0  # (N,4)

        # construct sampling grid for grid_sample
        if patch_px == 1:
            # sample single point per wheel -> flatten all N*4 samples into one grid_sample call
            # Reshape to (N*4,) then sample from (1,1,H,W) with grid (1,1,N*4,2)
            pix_x_flat = pix_x.reshape(-1)  # (N*4,)
            pix_y_flat = pix_y.reshape(-1)  # (N*4,)
            x_norm = (pix_x_flat / (W - 1)) * 2.0 - 1.0
            y_norm = (pix_y_flat / (H - 1)) * 2.0 - 1.0
            grid = torch.stack((x_norm, y_norm), dim=-1).unsqueeze(0).unsqueeze(0)  # (1,1,N*4,2)
            samples = F_torch.grid_sample(map_single, grid, mode='nearest', padding_mode='border', align_corners=False)
            wheel_heights = samples.squeeze(0).squeeze(0).squeeze(0).reshape(N, 4)  # (N,4)
        else:
            # generate offsets grid in pixels around center and average
            half = (patch_px - 1) / 2.0
            offsets_patch = torch.linspace(-half, half, steps=patch_px, device=device)
            off_x, off_y = torch.meshgrid(offsets_patch, offsets_patch, indexing='ij')
            off_x = off_x.reshape(-1)  # (K,)
            off_y = off_y.reshape(-1)
            K = off_x.shape[0]

            # expand centers to include offsets -> (N,4,K)
            pix_x_exp = pix_x.unsqueeze(-1) + off_x.view(1, 1, K)
            pix_y_exp = pix_y.unsqueeze(-1) + off_y.view(1, 1, K)

            # flatten all samples into one grid_sample call -> (1, 1, N*4*K, 2)
            pix_x_flat = pix_x_exp.reshape(-1)  # (N*4*K,)
            pix_y_flat = pix_y_exp.reshape(-1)
            x_norm = (pix_x_flat / (W - 1)) * 2.0 - 1.0
            y_norm = (pix_y_flat / (H - 1)) * 2.0 - 1.0
            grid = torch.stack((x_norm, y_norm), dim=-1).unsqueeze(0).unsqueeze(0)  # (1,1,N*4*K,2)
            samples = F_torch.grid_sample(map_single, grid, mode='bilinear', padding_mode='border', align_corners=True)
            samples = samples.squeeze(0).squeeze(0).squeeze(0).reshape(N, 4, K)  # (N,4,K)
            wheel_heights = samples.mean(dim=2)  # (N,4)

        # compute roll/pitch from wheel heights (torch on device)
        fl_h = wheel_heights[:, 0]
        fr_h = wheel_heights[:, 1]
        rl_h = wheel_heights[:, 2]
        rr_h = wheel_heights[:, 3]

        # Fallback to input z values for wheels that are NaN (matches get_patch_height behavior)
        fl_h = torch.where(torch.isnan(fl_h), t_zs, fl_h)
        fr_h = torch.where(torch.isnan(fr_h), t_zs, fr_h)
        rl_h = torch.where(torch.isnan(rl_h), t_zs, rl_h)
        rr_h = torch.where(torch.isnan(rr_h), t_zs, rr_h)
        
        left_avg_t = (fl_h + rl_h) / 2.0
        right_avg_t = (fr_h + rr_h) / 2.0
        front_avg_t = (fl_h + fr_h) / 2.0
        rear_avg_t = (rl_h + rr_h) / 2.0
        avg_height = left_avg_t + right_avg_t + front_avg_t + rear_avg_t
        t_zs = (avg_height/4) + 0.152
        track_t = torch.tensor(float(self.track), device=device)
        wb_t = torch.tensor(float(self.wheelbase), device=device)

        # Keep original roll sign
        wheel_roll_t = torch.atan2((left_avg_t - right_avg_t), track_t)
        
        # Match single-pose pitch sign (front minus rear)
        # Flip sign so positive pitch matches quaternion_to_pitch (nose-up is negative in TF convention)
        wheel_pitch_t = -torch.atan2((front_avg_t - rear_avg_t), wb_t)

        # default results use wheel-based values
        rolls_t = wheel_roll_t
        pitchs_t = wheel_pitch_t

        # Stack results in torch: [x, y, z, roll_rad, pitch_rad, yaw]
        results_t = torch.stack((t_xs, t_ys, t_zs, rolls_t, pitchs_t, t_yaws), dim=1)

        # Return requested type
        if return_torch:
            return results_t
        else:
            return results_t.cpu().numpy()

    def estimate_height(self, poses, patch_size=(0.05, 0.05), return_torch=True):
        """Estimate average height at a batch of poses using patch sampling.
        
        poses: numpy array or torch tensor of shape (N,2+). Expected columns:
            [x, y, ...] - only x and y are used, rest ignored.
        
        patch_size: tuple (width, height) in meters to average over at each pose.
                   Converted to pixels based on map resolution.
        
        return_torch: if True, return torch tensor; if False, return numpy array.
        
        Returns:
            heights: torch tensor or numpy array of shape (N,) with average heights at each pose.
        """
        device = self.device
        
        # Accept both numpy and torch, normalize to torch tensor on device
        if isinstance(poses, torch.Tensor):
            poses_t = poses.to(device)
        else:
            poses_t = torch.from_numpy(np.array(poses, dtype=np.float32)).to(device)
        
        if poses_t.ndim != 2 or poses_t.shape[1] < 2:
            raise ValueError('poses must be shape (N,2+) with at least x,y')
        
        N = poses_t.shape[0]
        
        # Extract only x, y from poses
        t_xs = poses_t[:, 0]
        t_ys = poses_t[:, 1]
        
        # Convert patch_size from meters to pixels
        if isinstance(patch_size, (tuple, list)):
            patch_w_p = max(1, int(round(patch_size[0] / self.map_res)))
            patch_h_p = max(1, int(round(patch_size[1] / self.map_res)))
        else:
            # If scalar, use as both width and height
            patch_w_p = max(1, int(round(patch_size / self.map_res)))
            patch_h_p = patch_w_p
        
        # Map metadata
        center = (int(self.map_shape[0] // 2), int(self.map_shape[1] // 2))
        origin_x = float(self.map_pose.position.x)
        origin_y = float(self.map_pose.position.y)
        res = float(self.map_res)
        H = int(self.map_shape[0])
        W = int(self.map_shape[1])
        
        # Convert pose world coordinates to pixel coordinates
        center_col = float(center[1])
        center_row = float(center[0])
        pix_x = center_col + (t_xs - origin_x) / res + 1.0  # (N,)
        pix_y = center_row + (t_ys - origin_y) / res + 1.0  # (N,)
        
        # Generate offsets grid in pixels around center
        half_w = (patch_w_p - 1) / 2.0
        half_h = (patch_h_p - 1) / 2.0
        offsets_x = torch.linspace(-half_w, half_w, steps=patch_w_p, device=device)
        offsets_y = torch.linspace(-half_h, half_h, steps=patch_h_p, device=device)
        off_x, off_y = torch.meshgrid(offsets_x, offsets_y, indexing='ij')
        off_x = off_x.reshape(-1)  # (K,)
        off_y = off_y.reshape(-1)
        K = off_x.shape[0]
        
        # Expand centers to include offsets -> (N, K)
        pix_x_exp = pix_x.unsqueeze(-1) + off_x.view(1, K)  # (N, K)
        pix_y_exp = pix_y.unsqueeze(-1) + off_y.view(1, K)  # (N, K)
        
        # Normalize to [-1, 1] for grid_sample
        x_norm = (pix_x_exp / (W - 1)) * 2.0 - 1.0
        y_norm = (pix_y_exp / (H - 1)) * 2.0 - 1.0
        
        # Create grid for grid_sample: (1, 1, N*K, 2) - single grid_sample call
        grid = torch.stack((x_norm, y_norm), dim=-1).reshape(-1, 2).unsqueeze(0).unsqueeze(0)  # (1, 1, N*K, 2)
        
        # Use cached elevation map tensor (1,1,H,W) on device
        map_single = self._map_single if hasattr(self, '_map_single') else None
        if map_single is None:
            if not isinstance(self.map_elevation, torch.Tensor):
                map_t = torch.from_numpy(self.map_elevation.astype(np.float32))
            else:
                map_t = self.map_elevation
            map_single = map_t.unsqueeze(0).unsqueeze(0).to(device)  # (1, 1, H, W)
        
        # Sample elevation at patch locations - single call for all N poses
        samples = F_torch.grid_sample(map_single, grid, mode='bilinear', padding_mode='border', align_corners=True)
        samples = samples.squeeze(0).squeeze(0).squeeze(0).reshape(N, K)  # (N, K)
        
        # Compute mean height per pose
        heights = samples.nanmean(dim=1)  # (N,)
        
        # Return requested type
        if return_torch:
            return heights
        else:
            return heights.cpu().numpy()

    #return hight at specific point on gridmap
    def get_pose_height(self, pose):
        # accept pose-like inputs
        px, py, pz, _ = self._extract_pose_components(pose)
        if not self.is_on_map((px, py)):
            return pz 

        t_x = int((px - self.map_pose.position.x)//self.map_res) + 1
        t_y = int((py - self.map_pose.position.y)//self.map_res) + 1
        center = (int(self.map_shape[0]//2), int(self.map_shape[1]//2))
        h = self.map_elevation[min((center[0] + t_y, (self.map_shape[1]-1)))][min((center[1] - t_x),(self.map_shape[0]-1))]
        
        # If height is NaN and we have a full elevation map, query it
        if np.isnan(h) and self.elevation_map is not None:
            h = self.elevation_map.get_patch(px, py, 1)[0, 0]
            if np.isnan(h):
                h = pz

        return h.item() if hasattr(h, 'item') else float(h)

    def get_patch_height(self, pose, patch_size):
        px, py, pz, _ = self._extract_pose_components(pose)
        if not self.is_on_map((px, py)):
            return pz

        # Use floating-point division to match batch method (not integer division //)
        t_x = (px - self.map_pose.position.x) / self.map_res + 1.0
        t_y = (py - self.map_pose.position.y) / self.map_res + 1.0
        center = (int(self.map_shape[0]//2), int(self.map_shape[1]//2))

        patch_size_p = max(1, int(round(patch_size / self.map_res)))

        ly, hy = center[0] + int(t_y - patch_size_p/2), center[0] + int(t_y + patch_size_p/2)
        lx, hx = center[1] + int(t_x - patch_size_p/2), center[1] + int(t_x + patch_size_p/2)

        ly = max(0, ly)
        lx = max(0, lx)
        hy = min(self.map_shape[0], hy)
        hx = min(self.map_shape[1], hx)

        if isinstance(self.map_elevation, torch.Tensor):
            h_patch = self.map_elevation[ly:hy, lx:hx].clone().detach().cpu().numpy()
        else:
            h_patch = self.map_elevation[ly:hy, lx:hx].copy()
        
        h = np.nanmean(h_patch)
        
        # If still all NaN, return original z
        if np.isnan(h):
            h = pz

        return h.item() if hasattr(h, 'item') else float(h)

    #return true is the pose is on the gridmap
    def is_on_map(self, pose):
        # accept pose-like or (x,y) tuple via duck typing
        try:
            if hasattr(pose, '__iter__') and not hasattr(pose, 'position'):
                x = float(pose[0]); y = float(pose[1])
            else:
                x, y = self._extract_xy(pose)
        except Exception:
            return False

        center = self.map_pose.position
        x_low = center.x - self.map_length/2
        x_high = center.x + self.map_length/2
        y_low = center.y - self.map_width/2
        y_high = center.y + self.map_width/2
        return (x_low < x) and (x < x_high) and (y_low < y) and (y < y_high)
    
    def is_on_dino_map(self, pose):
        """Check if pose is within DINO map bounds.
        
        Args:
            pose: Pose-like or (x,y) tuple
            
        Returns:
            bool: True if pose is within DINO map bounds
        """
        if self.dino_shape is None or self.dino_pose is None:
            return False
        
        try:
            if hasattr(pose, '__iter__') and not hasattr(pose, 'position'):
                x = float(pose[0]); y = float(pose[1])
            else:
                x, y = self._extract_xy(pose)
        except Exception:
            return False
        
        dino_length = self.dino_shape[0] * self.dino_res
        dino_width = self.dino_shape[1] * self.dino_res
        
        center = self.dino_pose.position
        x_low = center.x - dino_length/2
        x_high = center.x + dino_length/2
        y_low = center.y - dino_width/2
        y_high = center.y + dino_width/2
        
        return (x_low < x) and (x < x_high) and (y_low < y) and (y < y_high)
    
    def get_dino_patch(self, pose, patch_size, return_dict=True):
        """Get average DINOv3 features over a patch centered at pose.
        
        Args:
            pose: Pose-like input with x,y position (z and orientation ignored)
            patch_size: Size of patch in meters (scalar or tuple (width, height))
            return_dict: If True, return dict with keys 'dino_0', 'dino_1', 'dino_2'.
                        If False, return numpy array [dino_0, dino_1, dino_2]
        
        Returns:
            dict or numpy array: Average feature values in patch, or None if unavailable
        """
        # Check if DINO data is available
        if self.dino_shape is None or self.dino_pose is None:
            return None
        
        if self.map_dino_0 is None or self.map_dino_1 is None or self.map_dino_2 is None:
            return None
        
        # Extract x, y from pose
        px, py = self._extract_xy(pose)
        
        # Check if pose is on DINO map
        if not self.is_on_dino_map((px, py)):
            return None
        
        # Convert patch_size to pixels
        if isinstance(patch_size, (tuple, list)):
            patch_w_p = max(1, int(round(patch_size[0] / self.dino_res)))
            patch_h_p = max(1, int(round(patch_size[1] / self.dino_res)))
        else:
            patch_w_p = max(1, int(round(patch_size / self.dino_res)))
            patch_h_p = patch_w_p
        
        # Convert world coordinates to pixel coordinates in DINO map
        center = (int(self.dino_shape[0]//2), int(self.dino_shape[1]//2))
        t_x = (px - self.dino_pose.position.x) / self.dino_res + 1.0
        t_y = (py - self.dino_pose.position.y) / self.dino_res + 1.0
        
        # Calculate patch bounds in pixels
        ly = center[0] + int(t_y - patch_h_p/2)
        hy = center[0] + int(t_y + patch_h_p/2)
        lx = center[1] + int(t_x - patch_w_p/2)
        hx = center[1] + int(t_x + patch_w_p/2)
        
        # Clamp to map bounds
        ly = max(0, ly)
        lx = max(0, lx)
        hy = min(self.dino_shape[0], hy)
        hx = min(self.dino_shape[1], hx)
        
        # Extract patches and compute global average pooling (ignoring NaN)
        def global_avg_pool(layer):
            if isinstance(layer, torch.Tensor):
                patch = layer[ly:hy, lx:hx]
                # Global average pooling: exclude NaN/inf values
                valid_mask = torch.isfinite(patch)
                if valid_mask.sum() == 0:
                    # All values are NaN/inf, return 0.0
                    return 0.0
                # Average only valid (non-NaN) values
                mean_val = patch[valid_mask].mean()
                return mean_val.item() if hasattr(mean_val, 'item') else float(mean_val)
            else:
                patch = layer[ly:hy, lx:hx]
                # For numpy arrays, use nanmean which ignores NaN
                if patch.size == 0:
                    return 0.0
                mean_val = np.nanmean(patch)
                # If all values were NaN, nanmean returns NaN, replace with 0
                return float(mean_val) if np.isfinite(mean_val) else 0.0
        
        dino_0_mean = global_avg_pool(self.map_dino_0)
        dino_1_mean = global_avg_pool(self.map_dino_1)
        dino_2_mean = global_avg_pool(self.map_dino_2)
        
        if return_dict:
            return {
                'dino_0': dino_0_mean,
                'dino_1': dino_1_mean,
                'dino_2': dino_2_mean
            }
        else:
            return np.array([dino_0_mean, dino_1_mean, dino_2_mean], dtype=np.float32)

    def get_dino_patch_image(self, pose, patch_size, return_torch = False):
        """Get the actual DINOv3 feature patch as an RGB image.
        
        Args:
            pose: Pose-like input
            patch_size: Size of patch in meters (scalar or tuple)
            
        Returns:
            np.ndarray: (H, W, 3) image patch, or None if unavailable
        """
        if self.dino_shape is None or self.dino_pose is None:
            return None
        
        if self.map_dino_0 is None or self.map_dino_1 is None or self.map_dino_2 is None:
            return None
        
        # Extract x, y from pose
        try:
            px, py = self._extract_xy(pose)
        except Exception:
            return None
        
        # Check if pose is on DINO map
        if not self.is_on_dino_map((px, py)):
            return None
            
        # Convert patch_size to pixels
        if isinstance(patch_size, (tuple, list)):
            patch_w_p = max(1, int(round(patch_size[0] / self.dino_res)))
            patch_h_p = max(1, int(round(patch_size[1] / self.dino_res)))
        else:
            patch_w_p = max(1, int(round(patch_size / self.dino_res)))
            patch_h_p = patch_w_p
        
        # Convert world coordinates to pixel coordinates in DINO map
        center = (int(self.dino_shape[0]//2), int(self.dino_shape[1]//2))
        t_x = (px - self.dino_pose.position.x) / self.dino_res + 1.0
        t_y = (py - self.dino_pose.position.y) / self.dino_res + 1.0
        
        # Calculate patch bounds in pixels
        ly = center[0] + int(t_y - patch_h_p/2)
        hy = center[0] + int(t_y + patch_h_p/2)
        lx = center[1] + int(t_x - patch_w_p/2)
        hx = center[1] + int(t_x + patch_w_p/2)
        
        # Clamp to map bounds
        ly = max(0, ly)
        lx = max(0, lx)
        hy = min(self.dino_shape[0], hy)
        hx = min(self.dino_shape[1], hx)
        
        if hy <= ly or hx <= lx:
            return None

        # Extract patches
        def get_patch(layer):
            p = layer[ly:hy, lx:hx]
            if isinstance(p, torch.Tensor):
                return p
            return p.clone()

        p0 = get_patch(self.map_dino_0)
        p1 = get_patch(self.map_dino_1)
        p2 = get_patch(self.map_dino_2)

        patch_img_t = torch.stack([p0, p1, p2], dim=-1)  # (H, W, 3)

        
        # Stack into (H, W, 3)
        # patch_img = np.stack([p0, p1, p2], axis=-1)
        if not return_torch:
            patch_img = (patch_img_t * 255).to(torch.uint8)
            return patch_img.cpu().numpy()
        
        return patch_img_t
    
    def get_dino_features_batch(self, poses, patch_size=(0.05, 0.05), return_torch=True):
        """Get average DINOv3 features for a batch of poses using efficient torch operations.
        
        Args:
            poses: numpy array or torch tensor of shape (N,2+). Expected columns:
                  [x, y, ...] - only x and y are used, rest ignored.
            patch_size: tuple (width, height) in meters to average over at each pose.
            return_torch: if True, return torch tensor; if False, return numpy array.
        
        Returns:
            features: torch tensor or numpy array of shape (N, 3) with columns [dino_0, dino_1, dino_2]
                     Returns None if DINO data is not available.
        """
        # Check if DINO data is available
        if self.dino_shape is None or self.dino_pose is None:
            return None
        
        if self.map_dino_0 is None or self.map_dino_1 is None or self.map_dino_2 is None:
            return None
        
        device = self.device
        
        # Accept both numpy and torch, normalize to torch tensor on device
        if isinstance(poses, torch.Tensor):
            poses_t = poses.to(device)
        else:
            poses_t = torch.from_numpy(np.array(poses, dtype=np.float32)).to(device)
        
        if poses_t.ndim != 2 or poses_t.shape[1] < 2:
            raise ValueError('poses must be shape (N,2+) with at least x,y')
        
        N = poses_t.shape[0]
        
        # Extract only x, y from poses
        t_xs = poses_t[:, 0]
        t_ys = poses_t[:, 1]
        
        # Convert patch_size from meters to pixels
        if isinstance(patch_size, (tuple, list)):
            patch_w_p = max(1, int(round(patch_size[0] / self.dino_res)))
            patch_h_p = max(1, int(round(patch_size[1] / self.dino_res)))
        else:
            patch_w_p = max(1, int(round(patch_size / self.dino_res)))
            patch_h_p = patch_w_p
        
        # Map metadata
        center = (int(self.dino_shape[0] // 2), int(self.dino_shape[1] // 2))
        origin_x = float(self.dino_pose.position.x)
        origin_y = float(self.dino_pose.position.y)
        res = float(self.dino_res)
        H = int(self.dino_shape[0])
        W = int(self.dino_shape[1])
        
        # Convert pose world coordinates to pixel coordinates
        center_col = float(center[1])
        center_row = float(center[0])
        pix_x = center_col + (t_xs - origin_x) / res + 1.0  # (N,)
        pix_y = center_row + (t_ys - origin_y) / res + 1.0  # (N,)
        
        # Generate offsets grid in pixels around center
        half_w = (patch_w_p - 1) / 2.0
        half_h = (patch_h_p - 1) / 2.0
        offsets_x = torch.linspace(-half_w, half_w, steps=patch_w_p, device=device)
        offsets_y = torch.linspace(-half_h, half_h, steps=patch_h_p, device=device)
        off_x, off_y = torch.meshgrid(offsets_x, offsets_y, indexing='ij')
        off_x = off_x.reshape(-1)  # (K,)
        off_y = off_y.reshape(-1)
        K = off_x.shape[0]
        
        # Expand centers to include offsets -> (N, K)
        pix_x_exp = pix_x.unsqueeze(-1) + off_x.view(1, K)  # (N, K)
        pix_y_exp = pix_y.unsqueeze(-1) + off_y.view(1, K)  # (N, K)
        
        # Normalize to [-1, 1] for grid_sample
        x_norm = (pix_x_exp / (W - 1)) * 2.0 - 1.0
        y_norm = (pix_y_exp / (H - 1)) * 2.0 - 1.0
        
        # Create grid for grid_sample: (N, 1, K, 2)
        grid = torch.stack((x_norm, y_norm), dim=-1).unsqueeze(1)  # (N, 1, K, 2)
        
        # Helper function to perform global average pooling on a layer
        def global_avg_pool_batch(layer):
            if not isinstance(layer, torch.Tensor):
                layer_t = torch.from_numpy(layer.astype(np.float32))
            else:
                layer_t = layer
            
            # Expand to batch: (N, 1, H, W)
            map_batch = layer_t.unsqueeze(0).unsqueeze(0).to(device)
            map_batch = map_batch.expand(N, 1, H, W)
            
            # Sample at patch locations using bilinear interpolation
            samples = F_torch.grid_sample(map_batch, grid, mode='bilinear', 
                                         padding_mode='border', align_corners=True)
            samples = samples.squeeze(2).squeeze(1)  # (N, K)
            
            # Global average pooling: compute mean per pose, ignoring NaN values
            # For each pose, average all K sampled points, excluding NaN
            valid_mask = torch.isfinite(samples)  # (N, K)
            valid_counts = valid_mask.sum(dim=1, keepdim=True)  # (N, 1)
            
            # Replace NaN with 0 for summation, then divide by count of valid values
            samples_clean = torch.where(valid_mask, samples, torch.zeros_like(samples))
            sums = samples_clean.sum(dim=1)  # (N,)
            
            # Avoid division by zero: if no valid values, return 0
            means = torch.where(valid_counts.squeeze() > 0, 
                               sums / valid_counts.squeeze(), 
                               torch.zeros_like(sums))
            
            return means
        
        # Apply global average pooling to all three DINO layers
        dino_0_features = global_avg_pool_batch(self.map_dino_0)  # (N,)
        dino_1_features = global_avg_pool_batch(self.map_dino_1)  # (N,)
        dino_2_features = global_avg_pool_batch(self.map_dino_2)  # (N,)
        
        # Stack into (N, 3) tensor
        features_t = torch.stack((dino_0_features, dino_1_features, dino_2_features), dim=1)
        
        # Return requested type
        if return_torch:
            return features_t
        else:
            return features_t.cpu().numpy()

    def get_elevation_patch_image(self, pose, patch_size, return_torch=False):
        """Get the elevation map patch as a grayscale image.
        Args:
            pose: Pose-like input
            patch_size: Size of patch in meters (scalar or tuple)
        Returns:
            np.ndarray: (H, W) image patch, or None if unavailable
        """
        if self.map_elevation is None or self.elevation_map is None:
            return None
        try:
            px, py = self._extract_xy(pose)
        except Exception:
            return None
        if not self.is_on_map((px, py)):
            return None
        # Convert patch_size to pixels
        if isinstance(patch_size, (tuple, list)):
            patch_w_p = max(1, int(round(patch_size[0] / self.map_res)))
            patch_h_p = max(1, int(round(patch_size[1] / self.map_res)))
        else:
            patch_w_p = max(1, int(round(patch_size / self.map_res)))
            patch_h_p = patch_w_p
        # Convert world coordinates to pixel coordinates in elevation map
        center = (int(self.map_shape[0]//2), int(self.map_shape[1]//2))
        t_x = (px - self.map_pose.position.x) / self.map_res + 1.0
        t_y = (py - self.map_pose.position.y) / self.map_res + 1.0
        # Calculate patch bounds in pixels
        ly = center[0] + int(t_y - patch_h_p/2)
        hy = center[0] + int(t_y + patch_h_p/2)
        lx = center[1] + int(t_x - patch_w_p/2)
        hx = center[1] + int(t_x + patch_w_p/2)
        # Clamp to map bounds
        ly = max(0, ly)
        lx = max(0, lx)
        hy = min(self.map_shape[0], hy)
        hx = min(self.map_shape[1], hx)
        if hy <= ly or hx <= lx:
            return None
        patch = self.map_elevation[ly:hy, lx:hx]
        if return_torch:
            if not isinstance(patch, torch.Tensor):
                return torch.from_numpy(patch.astype(np.float32))
            return patch.float()
        else:
            if isinstance(patch, torch.Tensor):
                return patch.detach().cpu().numpy().astype(np.float32)
            return patch.astype(np.float32)

    def get_color_patch_image(self, pose, patch_size, layer_name="rgb", return_torch=False):
        """Get a packed color layer (like rgb or dino) patch as an RGB image."""
        try:
            px, py = self._extract_xy(pose)
        except Exception:
            return None
        if not self.is_on_map((px, py)):
            return None
            
        # Find index of the layer
        idx = None
        for i, name in enumerate(self.map.layers):
            if name == layer_name:
                idx = i
                break
        if idx is None:
            return None
            
        # Decode the packed layer to RGB channels
        r, g, b = self._decode_color_layer(self.map.data[idx].data, self.map_shape)
        rgb_t = torch.stack([r, g, b], dim=-1) # (H, W, 3)
        
        # Convert patch_size to pixels
        if isinstance(patch_size, (tuple, list)):
            patch_w_p = max(1, int(round(patch_size[0] / self.map_res)))
            patch_h_p = max(1, int(round(patch_size[1] / self.map_res)))
        else:
            patch_w_p = max(1, int(round(patch_size / self.map_res)))
            patch_h_p = patch_w_p
            
        # Convert world coordinates to pixel coordinates
        center = (int(self.map_shape[0]//2), int(self.map_shape[1]//2))
        t_x = (px - self.map_pose.position.x) / self.map_res + 1.0
        t_y = (py - self.map_pose.position.y) / self.map_res + 1.0
        
        # Calculate patch bounds in pixels
        ly = center[0] + int(t_y - patch_h_p/2)
        hy = center[0] + int(t_y + patch_h_p/2)
        lx = center[1] + int(t_x - patch_w_p/2)
        hx = center[1] + int(t_x + patch_w_p/2)
        
        # Clamp to map bounds
        ly = max(0, ly)
        lx = max(0, lx)
        hy = min(self.map_shape[0], hy)
        hx = min(self.map_shape[1], hx)
        
        if hy <= ly or hx <= lx:
            return None
            
        patch = rgb_t[ly:hy, lx:hx]
        
        if return_torch:
            return patch
        else:
            return (patch * 255.0).to(torch.uint8).cpu().numpy()