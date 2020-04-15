/**
 * @file frame_names.h
 *
 * Conventions:\n
 *   ENU is used wrt to some fixed global frame wherein:  +X -> East; +Y -> North; +Z -> Up\n
 *   FLU is used wrt to some relative body frame wherein: +X -> Front; +Y -> Left; +Z -> Up\n
 *   Level refers to the fact that the frame is "Level" to an imaginary ground plane.
 *       This means that by definition the Z axis is aligned with the the ENU frame.
 *       Level is a way to specify the X and Y axis in a local frame (e.g. body) while fixing\n
 *       the Z axis to align with the ENU frame.
 */

/// Frame wrt the world in ENU frame of reference.
/// Has some defined origin not necessarily associated with any asset
#define World_ENU "World_ENU"

// Frame wrt Asset's origin in the ENU orientation frame
#define Asset_ENU "Asset_ENU"

// Frame wrt Asset in its own origin and orientation
#define Asset_Frame "Asset_Frame"

// Frame wrt to an asset/feature whose orientation is level to the ground.
// This means that the X, Y, Z, and YAW coordinates are with respect to some Asset or Feature
//   while the pitch and roll are irrelevant and assumed to be 0 wrt to the Asset_Frame.
//#define Asset_Level "Asset_Level"
#define Asset_Level Asset_Frame

// Frame wrt to body of the UAV
#define body_FLU "body_FLU"

// Frame wrt to FLU body of the UAV that assumes the UAV is level to the ground.
// This means pitch and roll are irrelevant, i.e. assumed to be 0
// X -> front of UAV, Y -> left of UAV, Z -> up from UAV
#define body_Level_FLU "body_Level_FLU"
// For the time being body_FIXED and body_Level_FLU are synonymous
#define body_FIXED body_Level_FLU

// UAVs frame of reference in the World_ENU frame
#define drone_ENU "drone_ENU"

// UAVs frame of reference in the Asset_Frame frame
#define drone_Asset "drone_Asset"

// The ENU frame of reference coming from the FCU
#define FCU_ENU "FCU_ENU"

// Fused location frame
#define Fused_Frame "Fused_Frame"

// Velocity frame
#define VELOCITY_FLU_ID "VELOCITY_FLU"
#define VELOCITY_LEVEL_ID "VELOCITY_LEVEL"

// Map frame
#define Map_Frame "map"

// optitrack frame
#define optitrack_frame "optitrack_frame"

// bespoon frame
#define bespoon_frame "bespoon_frame"

// Interl RealSense Tracking Camera T265 frame
#define camera_odom_frame "camera_odom_frame"
