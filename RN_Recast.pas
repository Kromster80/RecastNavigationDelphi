//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//
{$POINTERMATH ON}

unit RN_Recast;
interface
uses
  Math, SysUtils, RN_Helper;

type
/// Recast log categories.
/// @see rcContext
  TrcLogCategory = (
  RC_LOG_PROGRESS = 1,  ///< A progress log entry.
  RC_LOG_WARNING,      ///< A warning log entry.
  RC_LOG_ERROR      ///< An error log entry.
  );

/// Recast performance timer categories.
/// @see rcContext
  TrcTimerLabel = 
(
  /// The user defined total time of the build.
  RC_TIMER_TOTAL,
  /// A user defined build time.
  RC_TIMER_TEMP,
  /// The time to rasterize the triangles. (See: #rcRasterizeTriangle)
  RC_TIMER_RASTERIZE_TRIANGLES,
  /// The time to build the compact heightfield. (See: #rcBuildCompactHeightfield)
  RC_TIMER_BUILD_COMPACTHEIGHTFIELD,
  /// The total time to build the contours. (See: #rcBuildContours)
  RC_TIMER_BUILD_CONTOURS,
  /// The time to trace the boundaries of the contours. (See: #rcBuildContours)
  RC_TIMER_BUILD_CONTOURS_TRACE,
  /// The time to simplify the contours. (See: #rcBuildContours)
  RC_TIMER_BUILD_CONTOURS_SIMPLIFY,
  /// The time to filter ledge spans. (See: #rcFilterLedgeSpans)
  RC_TIMER_FILTER_BORDER,
  /// The time to filter low height spans. (See: #rcFilterWalkableLowHeightSpans)
  RC_TIMER_FILTER_WALKABLE,
  /// The time to apply the median filter. (See: #rcMedianFilterWalkableArea)
  RC_TIMER_MEDIAN_AREA,
  /// The time to filter low obstacles. (See: #rcFilterLowHangingWalkableObstacles)
  RC_TIMER_FILTER_LOW_OBSTACLES,
  /// The time to build the polygon mesh. (See: #rcBuildPolyMesh)
  RC_TIMER_BUILD_POLYMESH,
  /// The time to merge polygon meshes. (See: #rcMergePolyMeshes)
  RC_TIMER_MERGE_POLYMESH,
  /// The time to erode the walkable area. (See: #rcErodeWalkableArea)
  RC_TIMER_ERODE_AREA,
  /// The time to mark a box area. (See: #rcMarkBoxArea)
  RC_TIMER_MARK_BOX_AREA,
  /// The time to mark a cylinder area. (See: #rcMarkCylinderArea)
  RC_TIMER_MARK_CYLINDER_AREA,
  /// The time to mark a convex polygon area. (See: #rcMarkConvexPolyArea)
  RC_TIMER_MARK_CONVEXPOLY_AREA,
  /// The total time to build the distance field. (See: #rcBuildDistanceField)
  RC_TIMER_BUILD_DISTANCEFIELD,
  /// The time to build the distances of the distance field. (See: #rcBuildDistanceField)
  RC_TIMER_BUILD_DISTANCEFIELD_DIST,
  /// The time to blur the distance field. (See: #rcBuildDistanceField)
  RC_TIMER_BUILD_DISTANCEFIELD_BLUR,
  /// The total time to build the regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
  RC_TIMER_BUILD_REGIONS,
  /// The total time to apply the watershed algorithm. (See: #rcBuildRegions)
  RC_TIMER_BUILD_REGIONS_WATERSHED,
  /// The time to expand regions while applying the watershed algorithm. (See: #rcBuildRegions)
  RC_TIMER_BUILD_REGIONS_EXPAND,
  /// The time to flood regions while applying the watershed algorithm. (See: #rcBuildRegions)
  RC_TIMER_BUILD_REGIONS_FLOOD,
  /// The time to filter out small regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
  RC_TIMER_BUILD_REGIONS_FILTER,
  /// The time to build heightfield layers. (See: #rcBuildHeightfieldLayers)
  RC_TIMER_BUILD_LAYERS,
  /// The time to build the polygon mesh detail. (See: #rcBuildPolyMeshDetail)
  RC_TIMER_BUILD_POLYMESHDETAIL,
  /// The time to merge polygon mesh details. (See: #rcMergePolyMeshDetails)
  RC_TIMER_MERGE_POLYMESHDETAIL,
  /// The maximum number of timers.  (Used for iterating timers.)
  RC_MAX_TIMERS
);

/// Provides an interface for optional logging and performance tracking of the Recast
/// build process.
/// @ingroup recast
  TrcContext = class
  public
    /// Contructor.
    ///  @param[in]    state  TRUE if the logging and performance timers should be enabled.  [Default: true]
    constructor Create(state: Boolean = true);//: m_logEnabled(state), m_timerEnabled(state) {}

    /// Enables or disables logging.
    ///  @param[in]    state  TRUE if logging should be enabled.
    procedure enableLog(state: Boolean); { m_logEnabled = state; }

    /// Clears all log entries.
    procedure resetLog(); { if (m_logEnabled) doResetLog(); }

    /// Logs a message.
    ///  @param[in]    category  The category of the message.
    ///  @param[in]    format    The message.
    procedure log(category: TrcLogCategory; msg: string);

    /// Enables or disables the performance timers.
    ///  @param[in]    state  TRUE if timers should be enabled.
    procedure enableTimer(state: Boolean); { m_timerEnabled = state; }

    /// Clears all peformance timers. (Resets all to unused.)
    procedure resetTimers(); { if (m_timerEnabled) doResetTimers(); }

    /// Starts the specified performance timer.
    ///  @param  label  The category of timer.
    procedure startTimer(aLabel: TrcTimerLabel); { if (m_timerEnabled) doStartTimer(label); }

    /// Stops the specified performance timer.
    ///  @param  label  The category of the timer.
    procedure stopTimer(aLabel: TrcTimerLabel); { if (m_timerEnabled) doStopTimer(label); }

    /// Returns the total accumulated time of the specified performance timer.
    ///  @param  label  The category of the timer.
    ///  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
    function getAccumulatedTime(aLabel: TrcTimerLabel): Integer; { return m_timerEnabled ? doGetAccumulatedTime(label) : -1; }
  protected
    /// True if logging is enabled.
    m_logEnabled: Boolean;

    /// True if the performance timers are enabled.
    m_timerEnabled: Boolean;

    /// Clears all log entries.
    procedure doResetLog(); virtual; abstract;

    /// Logs a message.
    ///  @param[in]    category  The category of the message.
    ///  @param[in]    msg      The formatted message.
    ///  @param[in]    len      The length of the formatted message.
    procedure doLog(category: TrcLogCategory; msg: string); virtual; abstract;

    /// Clears all timers. (Resets all to unused.)
    procedure doResetTimers(); virtual; abstract;

    /// Starts the specified performance timer.
    ///  @param[in]    label  The category of timer.
    procedure doStartTimer(const &label: TrcTimerLabel); virtual; abstract;

    /// Stops the specified performance timer.
    ///  @param[in]    label  The category of the timer.
    procedure doStopTimer(const &label: TrcTimerLabel); virtual; abstract;

    /// Returns the total accumulated time of the specified performance timer.
    ///  @param[in]    label  The category of the timer.
    ///  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
    function doGetAccumulatedTime(const &label: TrcTimerLabel): Integer; virtual; { return -1; }
  end;

/// Specifies a configuration to use when performing Recast builds.
/// @ingroup recast
  TrcConfig = record
    /// The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
    width: Integer;

    /// The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
    height: Integer;

    /// The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx]
    tileSize: Integer;

    /// The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
    borderSize: Integer;

    /// The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu]
    cs: Single;

    /// The y-axis cell size to use for fields. [Limit: > 0] [Units: wu]
    ch: Single;

    /// The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
    bmin: array [0..2] of Single;

    /// The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
    bmax: array [0..2] of Single;

    /// The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees]
    walkableSlopeAngle: Single;

    /// Minimum floor to 'ceiling' height that will still allow the floor area to
    /// be considered walkable. [Limit: >= 3] [Units: vx]
    walkableHeight: Integer;

    /// Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
    walkableClimb: Integer;

    /// The distance to erode/shrink the walkable area of the heightfield away from
    /// obstructions.  [Limit: >=0] [Units: vx]
    walkableRadius: Integer;

    /// The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx]
    maxEdgeLen: Integer;

    /// The maximum distance a simplfied contour's border edges should deviate
    /// the original raw contour. [Limit: >=0] [Units: vx]
    maxSimplificationError: Single;

    /// The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx]
    minRegionArea: Integer;

    /// Any regions with a span count smaller than this value will, if possible,
    /// be merged with larger regions. [Limit: >=0] [Units: vx]
    mergeRegionArea: Integer;

    /// The maximum number of vertices allowed for polygons generated during the
    /// contour to polygon conversion process. [Limit: >= 3]
    maxVertsPerPoly: Integer;

    /// Sets the sampling distance to use when generating the detail mesh.
    /// (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu]
    detailSampleDist: Single;

    /// The maximum distance the detail mesh surface should deviate from heightfield
    /// data. (For height detail only.) [Limit: >=0] [Units: wu]
    detailSampleMaxError: Single;
  end;

/// Defines the number of bits allocated to rcSpan::smin and rcSpan::smax.
const RC_SPAN_HEIGHT_BITS = 13;
/// Defines the maximum value for rcSpan::smin and rcSpan::smax.
const RC_SPAN_MAX_HEIGHT = 8191;//(1<<RC_SPAN_HEIGHT_BITS)-1;

/// The number of spans allocated per span spool.
/// @see rcSpanPool
const RC_SPANS_PER_POOL = 2048;

type
  /// Represents a span in a heightfield.
  /// @see rcHeightfield
  PrcSpan = ^TrcSpan;
  TrcSpan = record
    smin: Word; //: 13;      ///< The lower limit of the span. [Limit: < #smax]
    smax: Word; //: 13;      ///< The upper limit of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT]
    area: Byte; //: 6;      ///< The area id assigned to the span.
    next: PrcSpan;          ///< The next span higher up in column.
  end;

  /// A memory pool used for quick allocation of spans within a heightfield.
  /// @see rcHeightfield
  PrcSpanPool = ^TrcSpanPool;
  TrcSpanPool = record
    next: PrcSpanPool;          ///< The next span pool.
    items: array [0..RC_SPANS_PER_POOL-1] of TrcSpan;  ///< Array of spans in the pool.
  end;

  /// A dynamic heightfield representing obstructed space.
  /// @ingroup recast
  TrcHeightfield = record
    width: Integer;      ///< The width of the heightfield. (Along the x-axis in cell units.)
    height: Integer;      ///< The height of the heightfield. (Along the z-axis in cell units.)
    bmin: array [0..2] of Single;    ///< The minimum bounds in world space. [(x, y, z)]
    bmax: array [0..2] of Single;    ///< The maximum bounds in world space. [(x, y, z)]
    cs: Single;      ///< The size of each cell. (On the xz-plane.)
    ch: Single;      ///< The height of each cell. (The minimum increment along the y-axis.)
    spans: array of PrcSpan;    ///< Heightfield of spans (width*height).
    pools: PrcSpanPool;  ///< Linked list of span pools.
    freelist: PrcSpan;  ///< The next free span.
  end;
  PrcHeightfield = ^TrcHeightfield;

  /// Provides information on the content of a cell column in a compact heightfield.
  TrcCompactCell = record
    index: Cardinal;// : 24;  ///< Index to the first span in the column.
    count: Byte;// : 8;    ///< Number of spans in the column.
  end;
  PrcCompactCell = ^TrcCompactCell;

  /// Represents a span of unobstructed space within a compact heightfield.
  TrcCompactSpan = record
    y: Word;      ///< The lower extent of the span. (Measured from the heightfield's base.)
    reg: Word;      ///< The id of the region the span belongs to. (Or zero if not in a region.)
    con: Cardinal;// : 24;    ///< Packed neighbor connection data.
    h: Byte;// : 8;      ///< The height of the span.  (Measured from #y.)
  end;
  PrcCompactSpan = ^TrcCompactSpan;

  /// A compact, static heightfield representing unobstructed space.
  /// @ingroup recast
  TrcCompactHeightfield = record
    width: Integer;          ///< The width of the heightfield. (Along the x-axis in cell units.)
    height: Integer;          ///< The height of the heightfield. (Along the z-axis in cell units.)
    spanCount: Integer;        ///< The number of spans in the heightfield.
    walkableHeight: Integer;      ///< The walkable height used during the build of the field.  (See: rcConfig::walkableHeight)
    walkableClimb: Integer;      ///< The walkable climb used during the build of the field. (See: rcConfig::walkableClimb)
    borderSize: Integer;        ///< The AABB border size used during the build of the field. (See: rcConfig::borderSize)
    maxDistance: Word;  ///< The maximum distance value of any span within the field.
    maxRegions: Word;  ///< The maximum region id of any span within the field.
    bmin: array [0..2] of Single;        ///< The minimum bounds in world space. [(x, y, z)]
    bmax: array [0..2] of Single;        ///< The maximum bounds in world space. [(x, y, z)]
    cs: Single;          ///< The size of each cell. (On the xz-plane.)
    ch: Single;          ///< The height of each cell. (The minimum increment along the y-axis.)
    cells: array of TrcCompactCell;    ///< Array of cells. [Size: #width*#height]
    spans: array of TrcCompactSpan;    ///< Array of spans. [Size: #spanCount]
    dist: PWord;    ///< Array containing border distance data. [Size: #spanCount]
    areas: PByte;    ///< Array containing area id data. [Size: #spanCount]
  end;
  PrcCompactHeightfield = ^TrcCompactHeightfield;

  /// Represents a heightfield layer within a layer set.
  /// @see rcHeightfieldLayerSet
  TrcHeightfieldLayer = record
    bmin: array [0..2] of Single;        ///< The minimum bounds in world space. [(x, y, z)]
    bmax: array [0..2] of Single;        ///< The maximum bounds in world space. [(x, y, z)]
    cs: Single;          ///< The size of each cell. (On the xz-plane.)
    ch: Single;          ///< The height of each cell. (The minimum increment along the y-axis.)
    width: Integer;          ///< The width of the heightfield. (Along the x-axis in cell units.)
    height: Integer;          ///< The height of the heightfield. (Along the z-axis in cell units.)
    minx: Integer;          ///< The minimum x-bounds of usable data.
    maxx: Integer;          ///< The maximum x-bounds of usable data.
    miny: Integer;          ///< The minimum y-bounds of usable data. (Along the z-axis.)
    maxy: Integer;          ///< The maximum y-bounds of usable data. (Along the z-axis.)
    hmin: Integer;          ///< The minimum height bounds of usable data. (Along the y-axis.)
    hmax: Integer;          ///< The maximum height bounds of usable data. (Along the y-axis.)
    heights: PByte;    ///< The heightfield. [Size: width * height]
    areas: PByte;    ///< Area ids. [Size: Same as #heights]
    cons: PByte;    ///< Packed neighbor connection information. [Size: Same as #heights]
  end;
  PrcHeightfieldLayer = ^TrcHeightfieldLayer;

  /// Represents a set of heightfield layers.
  /// @ingroup recast
  /// @see rcAllocHeightfieldLayerSet, rcFreeHeightfieldLayerSet
  TrcHeightfieldLayerSet = record
    layers: array of TrcHeightfieldLayer;      ///< The layers in the set. [Size: #nlayers]
    nlayers: Integer;            ///< The number of layers in the set.
  end;

  /// Represents a simple, non-overlapping contour in field space.
  TrcContour = record
    verts: PInteger;      ///< Simplified contour vertex and connection data. [Size: 4 * #nverts]
    nverts: Integer;      ///< The number of vertices in the simplified contour.
    rverts: PInteger;    ///< Raw contour vertex and connection data. [Size: 4 * #nrverts]
    nrverts: Integer;    ///< The number of vertices in the raw contour.
    reg: Word;  ///< The region id of the contour.
    area: Byte;  ///< The area id of the contour.
  end;
  PrcContour = ^TrcContour;

  /// Represents a group of related contours.
  /// @ingroup recast
  TrcContourSet = record
    conts: array of TrcContour;  ///< An array of the contours in the set. [Size: #nconts]
    nconts: Integer;      ///< The number of contours in the set.
    bmin: array [0..2] of Single;    ///< The minimum bounds in world space. [(x, y, z)]
    bmax: array [0..2] of Single;    ///< The maximum bounds in world space. [(x, y, z)]
    cs: Single;      ///< The size of each cell. (On the xz-plane.)
    ch: Single;      ///< The height of each cell. (The minimum increment along the y-axis.)
    width: Integer;      ///< The width of the set. (Along the x-axis in cell units.)
    height: Integer;      ///< The height of the set. (Along the z-axis in cell units.)
    borderSize: Integer;    ///< The AABB border size used to generate the source data from which the contours were derived.
  end;
  PrcContourSet = ^TrcContourSet;

  /// Represents a polygon mesh suitable for use in building a navigation mesh.
  /// @ingroup recast
  TrcPolyMesh = record
    verts: PWord;  ///< The mesh vertices. [Form: (x, y, z) * #nverts]
    polys: PWord;  ///< Polygon and neighbor data. [Length: #maxpolys * 2 * #nvp]
    regs: PWord;  ///< The region id assigned to each polygon. [Length: #maxpolys]
    flags: PWord;  ///< The user defined flags for each polygon. [Length: #maxpolys]
    areas: PByte;  ///< The area id assigned to each polygon. [Length: #maxpolys]
    nverts: Integer;        ///< The number of vertices.
    npolys: Integer;        ///< The number of polygons.
    maxpolys: Integer;      ///< The number of allocated polygons.
    nvp: Integer;        ///< The maximum number of vertices per polygon.
    bmin: array [0..2] of Single;    ///< The minimum bounds in world space. [(x, y, z)]
    bmax: array [0..2] of Single;    ///< The maximum bounds in world space. [(x, y, z)]
    cs: Single;        ///< The size of each cell. (On the xz-plane.)
    ch: Single;        ///< The height of each cell. (The minimum increment along the y-axis.)
    borderSize: Integer;      ///< The AABB border size used to generate the source data from which the mesh was derived.
  end;
  PrcPolyMesh = ^TrcPolyMesh;

  /// Contains triangle meshes that represent detailed height data associated
  /// with the polygons in its associated polygon mesh object.
  /// @ingroup recast
  TrcPolyMeshDetail = record
    meshes: PCardinal;  ///< The sub-mesh data. [Size: 4*#nmeshes]
    verts: PSingle;      ///< The mesh vertices. [Size: 3*#nverts]
    tris: PByte;  ///< The mesh triangles. [Size: 4*#ntris]
    nmeshes: Integer;      ///< The number of sub-meshes defined by #meshes.
    nverts: Integer;        ///< The number of vertices in #verts.
    ntris: Integer;        ///< The number of triangles in #tris.
  end;
  PrcPolyMeshDetail = ^TrcPolyMeshDetail;

/// @name Allocation Functions
/// Functions used to allocate and de-allocate Recast objects.
/// @see rcAllocSetCustom
/// @{

/// Allocates a heightfield object using the Recast allocator.
///  @return A heightfield that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcCreateHeightfield, rcFreeHeightField
//rcHeightfield* rcAllocHeightfield();

/// Frees the specified heightfield object using the Recast allocator.
///  @param[in]    hf  A heightfield allocated using #rcAllocHeightfield
///  @ingroup recast
///  @see rcAllocHeightfield
//void rcFreeHeightField(rcHeightfield* hf);

/// Allocates a compact heightfield object using the Recast allocator.
///  @return A compact heightfield that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildCompactHeightfield, rcFreeCompactHeightfield
//rcCompactHeightfield* rcAllocCompactHeightfield();

/// Frees the specified compact heightfield object using the Recast allocator.
///  @param[in]    chf    A compact heightfield allocated using #rcAllocCompactHeightfield
///  @ingroup recast
///  @see rcAllocCompactHeightfield
//void rcFreeCompactHeightfield(rcCompactHeightfield* chf);

/// Allocates a heightfield layer set using the Recast allocator.
///  @return A heightfield layer set that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildHeightfieldLayers, rcFreeHeightfieldLayerSet
//rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet();

/// Frees the specified heightfield layer set using the Recast allocator.
///  @param[in]    lset  A heightfield layer set allocated using #rcAllocHeightfieldLayerSet
///  @ingroup recast
///  @see rcAllocHeightfieldLayerSet
//void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* lset);

/// Allocates a contour set object using the Recast allocator.
///  @return A contour set that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildContours, rcFreeContourSet
//rcContourSet* rcAllocContourSet();

/// Frees the specified contour set using the Recast allocator.
///  @param[in]    cset  A contour set allocated using #rcAllocContourSet
///  @ingroup recast
///  @see rcAllocContourSet
//void rcFreeContourSet(rcContourSet* cset);

/// Allocates a polygon mesh object using the Recast allocator.
///  @return A polygon mesh that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildPolyMesh, rcFreePolyMesh
//rcPolyMesh* rcAllocPolyMesh();

/// Frees the specified polygon mesh using the Recast allocator.
///  @param[in]    pmesh  A polygon mesh allocated using #rcAllocPolyMesh
///  @ingroup recast
///  @see rcAllocPolyMesh
//void rcFreePolyMesh(rcPolyMesh* pmesh);

/// Allocates a detail mesh object using the Recast allocator.
///  @return A detail mesh that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildPolyMeshDetail, rcFreePolyMeshDetail
//rcPolyMeshDetail* rcAllocPolyMeshDetail();

/// Frees the specified detail mesh using the Recast allocator.
///  @param[in]    dmesh  A detail mesh allocated using #rcAllocPolyMeshDetail
///  @ingroup recast
///  @see rcAllocPolyMeshDetail
//void rcFreePolyMeshDetail(rcPolyMeshDetail* dmesh);

/// @}

/// Heighfield border flag.
/// If a heightfield region ID has this bit set, then the region is a border
/// region and its spans are considered unwalkable.
/// (Used during the region and contour build process.)
/// @see rcCompactSpan::reg
const RC_BORDER_REG = $8000;

/// Border vertex flag.
/// If a region ID has this bit set, then the associated element lies on
/// a tile border. If a contour vertex's region ID has this bit set, the
/// vertex will later be removed in order to match the segments and vertices
/// at tile boundaries.
/// (Used during the build process.)
/// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
const RC_BORDER_VERTEX = $10000;

/// Area border flag.
/// If a region ID has this bit set, then the associated element lies on
/// the border of an area.
/// (Used during the region and contour build process.)
/// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
const RC_AREA_BORDER = $20000;

const
  /// Contour build flags.
  /// @see rcBuildContours
  //TrcBuildContoursFlags =
  //(
    RC_CONTOUR_TESS_WALL_EDGES = $01;  ///< Tessellate solid (impassable) edges during contour simplification.
    RC_CONTOUR_TESS_AREA_EDGES = $02;  ///< Tessellate edges between areas during contour simplification.
  //);

/// Applied to the region id field of contour vertices in order to extract the region id.
/// The region id field of a vertex may have several flags applied to it.  So the
/// fields value can't be used directly.
/// @see rcContour::verts, rcContour::rverts
const RC_CONTOUR_REG_MASK = $ffff;

/// An value which indicates an invalid index within a mesh.
/// @note This does not necessarily indicate an error.
/// @see rcPolyMesh::polys
const RC_MESH_NULL_IDX = $ffff;

/// Represents the null area.
/// When a data element is given this value it is considered to no longer be 
/// assigned to a usable area.  (E.g. It is unwalkable.)
const RC_NULL_AREA = 0;

/// The default area id used to indicate a walkable polygon.
/// This is also the maximum allowed area id, and the only non-null area id
/// recognized by some steps in the build process.
const RC_WALKABLE_AREA = 63;

/// The value returned by #rcGetCon if the specified direction is not connected
/// to another span. (Has no neighbor.)
const RC_NOT_CONNECTED = $3f;

/// @}
/// @name Heightfield Functions
/// @see rcHeightfield
/// @{

/// Calculates the bounding box of an array of vertices.
///  @ingroup recast
///  @param[in]    verts  An array of vertices. [(x, y, z) * @p nv]
///  @param[in]    nv    The number of vertices in the @p verts array.
///  @param[out]  bmin  The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
///  @param[out]  bmax  The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
procedure rcCalcBounds(const verts: PSingle; nv: Integer; bmin, bmax: PSingle);

/// Calculates the grid size based on the bounding box and grid cell size.
///  @ingroup recast
///  @param[in]    bmin  The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
///  @param[in]    bmax  The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
///  @param[in]    cs    The xz-plane cell size. [Limit: > 0] [Units: wu]
///  @param[out]  w    The width along the x-axis. [Limit: >= 0] [Units: vx]
///  @param[out]  h    The height along the z-axis. [Limit: >= 0] [Units: vx]
procedure rcCalcGridSize(const bmin, bmax: PSingle; cs: Single; w,h: PInteger);

/// Initializes a new heightfield.
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in,out]  hf    The allocated heightfield to initialize.
///  @param[in]    width  The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
///  @param[in]    height  The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
///  @param[in]    bmin  The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
///  @param[in]    bmax  The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
///  @param[in]    cs    The xz-plane cell size to use for the field. [Limit: > 0] [Units: wu]
///  @param[in]    ch    The y-axis cell size to use for field. [Limit: > 0] [Units: wu]
function rcCreateHeightfield(ctx: TrcContext; var hf: TrcHeightfield; width, height: Integer;
             const bmin, bmax: PSingle;
             cs, ch: Single): Boolean;

/// Sets the area id of all triangles with a slope below the specified value
/// to #RC_WALKABLE_AREA.
///  @ingroup recast
///  @param[in,out]  ctx          The build context to use during the operation.
///  @param[in]    walkableSlopeAngle  The maximum slope that is considered walkable.
///                    [Limits: 0 <= value < 90] [Units: Degrees]
///  @param[in]    verts        The vertices. [(x, y, z) * @p nv]
///  @param[in]    nv          The number of vertices.
///  @param[in]    tris        The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]    nt          The number of triangles.
///  @param[out]  areas        The triangle area ids. [Length: >= @p nt]
procedure rcMarkWalkableTriangles(ctx: TrcContext; const walkableSlopeAngle: Single; const verts: PSingle; nv: Integer;
               const tris: PInteger; nt: Integer; areas: PByte);

/// Sets the area id of all triangles with a slope greater than or equal to the specified value to #RC_NULL_AREA.
///  @ingroup recast
///  @param[in,out]  ctx          The build context to use during the operation.
///  @param[in]    walkableSlopeAngle  The maximum slope that is considered walkable.
///                    [Limits: 0 <= value < 90] [Units: Degrees]
///  @param[in]    verts        The vertices. [(x, y, z) * @p nv]
///  @param[in]    nv          The number of vertices.
///  @param[in]    tris        The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]    nt          The number of triangles.
///  @param[out]  areas        The triangle area ids. [Length: >= @p nt]
procedure rcClearUnwalkableTriangles(ctx: TrcContext; const walkableSlopeAngle: Single; const verts: PSingle; const nv: Integer;
                const tris: PInteger; nt: Integer; out areas: PInteger);

/// Returns the number of spans contained in the specified heightfield.
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in]    hf    An initialized heightfield.
///  @returns The number of spans in the heightfield.
function rcGetHeightFieldSpanCount(ctx: TrcContext; hf: PrcHeightfield): Integer;

/// @}
/// @name Compact Heightfield Functions
/// @see rcCompactHeightfield
/// @{

/// Builds a compact heightfield representing open space, from a heightfield representing solid space.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in]    walkableHeight  Minimum floor to 'ceiling' height that will still allow the floor area
///                  to be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[in]    walkableClimb  Maximum ledge height that is considered to still be traversable.
///                  [Limit: >=0] [Units: vx]
///  @param[in]    hf        The heightfield to be compacted.
///  @param[out]  chf        The resulting compact heightfield. (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
function rcBuildCompactHeightfield(ctx: TrcContext; const walkableHeight, walkableClimb: Integer;
                 hf: PrcHeightfield; chf: PrcCompactHeightfield): Boolean;

implementation
uses
  RN_RecastHelper;

procedure rcCalcBounds(const verts: PSingle; nv: Integer; bmin, bmax: PSingle);
var i: Integer; v: PSingle;
begin
  // Calculate bounding box.
  rcVcopy(bmin, verts);
  rcVcopy(bmax, verts);
  for i := 1 to nv - 1 do
  begin
    v := @verts[i*3];
    rcVmin(bmin, v);
    rcVmax(bmax, v);
  end;
end;

procedure rcCalcGridSize(const bmin, bmax: PSingle; cs: Single; w,h: PInteger);
begin
  w^ := Trunc((bmax[0] - bmin[0])/cs+0.5);
  h^ := Trunc((bmax[2] - bmin[2])/cs+0.5);
end;


/// @par
///
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcAllocHeightfield, rcHeightfield
function rcCreateHeightfield(ctx: TrcContext; var hf: TrcHeightfield; width, height: Integer;
             const bmin, bmax: PSingle;
             cs, ch: Single): Boolean;
begin
  //rcIgnoreUnused(ctx);

  hf.width := width;
  hf.height := height;
  rcVcopy(@hf.bmin, bmin);
  rcVcopy(@hf.bmax, bmax);
  hf.cs := cs;
  hf.ch := ch;
  SetLength(hf.spans, hf.width * hf.height);
  Result := true;
end;


procedure calcTriNormal(const v0, v1, v2: PSingle; norm: PSingle);
var e0, e1: array [0..2] of Single;
begin
  rcVsub(@e0, v1, v0);
  rcVsub(@e1, v2, v0);
  rcVcross(norm, @e0, @e1);
  rcVnormalize(norm);
end;

/// @par
///
/// Only sets the aread id's for the walkable triangles.  Does not alter the
/// area id's for unwalkable triangles.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
procedure rcMarkWalkableTriangles(ctx: TrcContext; const walkableSlopeAngle: Single; const verts: PSingle; nv: Integer;
               const tris: PInteger; nt: Integer; areas: PByte);
var walkableThr: Single; norm: array [0..2] of Single; i: Integer; tri: PInteger;
begin
  //rcIgnoreUnused(ctx);

  walkableThr := cos(walkableSlopeAngle/180*Pi);

  for i := 0 to nt - 1 do
  begin
    tri := @tris[i*3];
    calcTriNormal(@verts[tri[0]*3], @verts[tri[1]*3], @verts[tri[2]*3], @norm);
    // Check if the face is walkable.
    if (norm[1] > walkableThr) then
      areas[i] := RC_WALKABLE_AREA;
  end;
end;


/// @par
///
/// Only sets the aread id's for the unwalkable triangles.  Does not alter the
/// area id's for walkable triangles.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
procedure rcClearUnwalkableTriangles(ctx: TrcContext; const walkableSlopeAngle: Single; const verts: PSingle; const nv: Integer;
                const tris: PInteger; nt: Integer; out areas: PInteger);
var walkableThr: Single; norm: array [0..2] of Single; i: Integer; tri: PInteger;
begin
  //rcIgnoreUnused(ctx);

  walkableThr := cos(walkableSlopeAngle/180.0*PI);

  for i := 0 to nt - 1 do
  begin
    tri := @tris[i*3];
    calcTriNormal(@verts[tri[0]*3], @verts[tri[1]*3], @verts[tri[2]*3], @norm[0]);
    // Check if the face is walkable.
    if (norm[1] <= walkableThr) then
      areas[i] := RC_NULL_AREA;
  end;
end;

function rcGetHeightFieldSpanCount(ctx: TrcContext; hf: PrcHeightfield): Integer;
var y,x,w,h,spanCount: Integer; s: PrcSpan;
begin
  w := hf.width;
  h := hf.height;
  spanCount := 0;
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      s := hf.spans[x + y*w];
      while s <> nil do
      begin
        if (s.area <> RC_NULL_AREA) then
          Inc(spanCount);
        s := s.next;
      end;
    end;
  end;
  Result := spanCount;
end;


/// @par
///
/// This is just the beginning of the process of fully building a compact heightfield.
/// Various filters may be applied applied, then the distance field and regions built.
/// E.g: #rcBuildDistanceField and #rcBuildRegions
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocCompactHeightfield, rcHeightfield, rcCompactHeightfield, rcConfig
function rcBuildCompactHeightfield(ctx: TrcContext; const walkableHeight, walkableClimb: Integer;
                 hf: PrcHeightfield; chf: PrcCompactHeightfield): Boolean;
const MAX_HEIGHT = $ffff;
const MAX_LAYERS = RC_NOT_CONNECTED-1;
var w,h,spanCount: Integer; idx,x,y: Integer; s: PrcSpan; c, nc: PrcCompactCell; bot,top: Integer;
tooHighNeighbour: Integer; nx,ny: Integer; cs, ncs: PrcCompactSpan;
i,dir,k: Integer; lidx: Integer;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

  w := hf.width;
  h := hf.height;
  spanCount := rcGetHeightFieldSpanCount(ctx, hf);

  // Fill in header.
  chf.width := w;
  chf.height := h;
  chf.spanCount := spanCount;
  chf.walkableHeight := walkableHeight;
  chf.walkableClimb := walkableClimb;
  chf.maxRegions := 0;
  rcVcopy(@chf.bmin, @hf.bmin);
  rcVcopy(@chf.bmax, @hf.bmax);
  chf.bmax[1] := chf.bmax[1] + walkableHeight*hf.ch;
  chf.cs := hf.cs;
  chf.ch := hf.ch;

  SetLength(chf.cells, w*h);
  SetLength(chf.spans, spanCount);
  GetMem(chf.areas, sizeof(Byte)*spanCount);
  FillChar(chf.areas[0], sizeof(Byte)*spanCount, RC_NULL_AREA);

  // Fill in cells and spans.
  idx := 0;
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      s := hf.spans[x + y*w];
      // If there are no spans at this cell, just leave the data to index=0, count=0.
      if (s = nil) then Continue;

      c := @chf.cells[x+y*w];
      c.index := idx;
      c.count := 0;
      while (s <> nil) do
      begin
        if (s.area <> RC_NULL_AREA) then
        begin
          bot := Integer(s.smax);
          if s.next <> nil then top := Integer(s.next.smin) else top := MAX_HEIGHT;
          chf.spans[idx].y := Word(rcClamp(bot, 0, $ffff));
          chf.spans[idx].h := Byte(rcClamp(top - bot, 0, $ff));
          chf.areas[idx] := s.area;
          Inc(idx);
          Inc(c.count);
        end;
        s := s.next;
      end;
    end;
  end;

  // Find neighbour connections.
  //const int MAX_LAYERS = RC_NOT_CONNECTED-1;
  tooHighNeighbour := 0;
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        cs := @chf.spans[i]; // s-> cs

        for dir := 0 to 3 do
        begin
          rcSetCon(cs, dir, RC_NOT_CONNECTED);
          nx := x + rcGetDirOffsetX(dir);
          ny := y + rcGetDirOffsetY(dir);
          // First check that the neighbour cell is in bounds.
          if (nx < 0) or (ny < 0) or (nx >= w) or (ny >= h) then
            Continue;

          // Iterate over all neighbour spans and check if any of the is
          // accessible from current cell.
          nc := @chf.cells[nx+ny*w];
          for k := nc.index to Integer(nc.index+nc.count) - 1 do
          begin
            ncs := @chf.spans[k]; // ns -> ncs
            bot := rcMax(cs.y, ncs.y);
            top := rcMin(cs.y+cs.h, ncs.y+ncs.h);

            // Check that the gap between the spans is walkable,
            // and that the climb height between the gaps is not too high.
            if ((top - bot) >= walkableHeight) and (Abs(ncs.y - cs.y) <= walkableClimb) then
            begin
              // Mark direction as walkable.
              lidx := k - nc.index;
              if (lidx < 0) or (lidx > MAX_LAYERS) then
              begin
                tooHighNeighbour := rcMax(tooHighNeighbour, lidx);
                Continue;
              end;
              rcSetCon(cs, dir, lidx);
              break;
            end;
          end;

        end;
      end;
    end;
  end;

  if (tooHighNeighbour > MAX_LAYERS) then
  begin
    ctx.log(RC_LOG_ERROR, Format('rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)',
         [tooHighNeighbour, MAX_LAYERS]));
  end;

  ctx.stopTimer(RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

  Result := True;
end;

{ TrcContext }
constructor TrcContext.Create(state: Boolean);
begin
  m_logEnabled := state;
  m_timerEnabled := state;
end;

function TrcContext.doGetAccumulatedTime(const &label: TrcTimerLabel): Integer;
begin
  Result := -1;
end;

procedure TrcContext.enableLog(state: Boolean);
begin
  m_logEnabled := state;
end;

procedure TrcContext.enableTimer(state: Boolean);
begin
  m_timerEnabled := state;
end;

function TrcContext.getAccumulatedTime(aLabel: TrcTimerLabel): Integer;
begin
  if m_timerEnabled then Result := doGetAccumulatedTime(alabel) else Result :=  -1;
end;

procedure TrcContext.log(category: TrcLogCategory; msg: string);
const MSG_SIZE = 512;
var len: Integer;
begin
  if (not m_logEnabled) then
    Exit;

  len := Length(msg);
  if (len >= MSG_SIZE) then
  begin
    len := MSG_SIZE-1;
    SetLength(msg, len);
  end;

  doLog(category, msg);
end;

procedure TrcContext.resetLog;
begin
  if (m_logEnabled) then doResetLog();
end;

procedure TrcContext.resetTimers;
begin
  if (m_timerEnabled) then doResetTimers();
end;

procedure TrcContext.startTimer(aLabel: TrcTimerLabel);
begin
  if (m_timerEnabled) then doStartTimer(aLabel);
end;

procedure TrcContext.stopTimer(aLabel: TrcTimerLabel);
begin
  if (m_timerEnabled) then doStopTimer(aLabel);
end;


end.
