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
unit RN_DetourNavMeshQuery;
interface
uses Classes, RN_DetourCommon, RN_DetourNavMesh, RN_DetourNavMeshHelper, RN_DetourNode, RN_DetourStatus;

type
  // Define DT_VIRTUAL_QUERYFILTER if you wish to derive a custom filter from dtQueryFilter.
  // On certain platforms indirect or virtual function call is expensive. The default
  // setting is to use non-virtual functions, the actual implementations of the functions
  // are declared as inline for maximum speed.

  //#define DT_VIRTUAL_QUERYFILTER 1

  /// Defines polygon filtering and traversal costs for navigation mesh query operations.
  /// @ingroup detour
  TdtQueryFilter = class
  private
    m_areaCost: array [0..DT_MAX_AREAS-1] of Single;    ///< Cost per area type. (Used by default implementation.)
    m_includeFlags: Word;    ///< Flags for polygons that can be visited. (Used by default implementation.)
    m_excludeFlags: Word;    ///< Flags for polygons that should not be visted. (Used by default implementation.)
  public
    constructor Create;

    /// Returns true if the polygon can be visited.  (I.e. Is traversable.)
    ///  @param[in]    ref    The reference id of the polygon test.
    ///  @param[in]    tile  The tile containing the polygon.
    ///  @param[in]    poly  The polygon to test.
    function passFilter(ref: TdtPolyRef; tile: PdtMeshTile; poly: PdtPoly): Boolean;

    /// Returns cost to move from the beginning to the end of a line segment
    /// that is fully contained within a polygon.
    ///  @param[in]    pa      The start position on the edge of the previous and current polygon. [(x, y, z)]
    ///  @param[in]    pb      The end position on the edge of the current and next polygon. [(x, y, z)]
    ///  @param[in]    prevRef    The reference id of the previous polygon. [opt]
    ///  @param[in]    prevTile  The tile containing the previous polygon. [opt]
    ///  @param[in]    prevPoly  The previous polygon. [opt]
    ///  @param[in]    curRef    The reference id of the current polygon.
    ///  @param[in]    curTile    The tile containing the current polygon.
    ///  @param[in]    curPoly    The current polygon.
    ///  @param[in]    nextRef    The refernece id of the next polygon. [opt]
    ///  @param[in]    nextTile  The tile containing the next polygon. [opt]
    ///  @param[in]    nextPoly  The next polygon. [opt]
    function getCost(pa, pb: PSingle;
            prevRef: TdtPolyRef; prevTile: PdtMeshTile; prevPoly: PdtPoly;
            curRef: TdtPolyRef; curTile: PdtMeshTile; curPoly: PdtPoly;
            nextRef: TdtPolyRef; nextTile: PdtMeshTile; nextPoly: PdtPoly): Single;

    /// @name Getters and setters for the default implementation data.
    ///@{

    /// Returns the traversal cost of the area.
    ///  @param[in]    i    The id of the area.
    /// @returns The traversal cost of the area.
    function getAreaCost(i: Integer): Single; { return m_areaCost[i]; }

    /// Sets the traversal cost of the area.
    ///  @param[in]    i    The id of the area.
    ///  @param[in]    cost  The new cost of traversing the area.
    procedure setAreaCost(i: Integer; cost: Single); { m_areaCost[i] = cost; }

    /// Returns the include flags for the filter.
    /// Any polygons that include one or more of these flags will be
    /// included in the operation.
    function getIncludeFlags(): Word; { return m_includeFlags; }

    /// Sets the include flags for the filter.
    /// @param[in]    flags  The new flags.
    procedure setIncludeFlags(flags: Word); { m_includeFlags = flags; }

    /// Returns the exclude flags for the filter.
    /// Any polygons that include one ore more of these flags will be
    /// excluded from the operation.
    function getExcludeFlags(): Word; { return m_excludeFlags; }

    /// Sets the exclude flags for the filter.
    /// @param[in]    flags    The new flags.
    procedure setExcludeFlags(flags: Word); { m_excludeFlags = flags; }

    ///@}
  end;

  /// Provides information about raycast hit
  /// filled by dtNavMeshQuery::raycast
  /// @ingroup detour
  PdtRaycastHit = ^TdtRaycastHit;
  TdtRaycastHit = record
    /// The hit parameter. (FLT_MAX if no wall hit.)
    t: Single;

    /// hitNormal  The normal of the nearest wall hit. [(x, y, z)]
    hitNormal: array [0..2] of Single;

    /// Pointer to an array of reference ids of the visited polygons. [opt]
    path: PdtPolyRef;

    /// The number of visited polygons. [opt]
    pathCount: Integer;

    /// The maximum number of polygons the @p path array can hold.
    maxPath: Integer;

    ///  The cost of the path until hit.
    pathCost: Single;
  end;


  TdtQueryData = record
    status: TdtStatus;
    lastBestNode: PdtNode;
    lastBestNodeCost: Single;
    startRef, endRef: TdtPolyRef;
    startPos, endPos: array [0..2] of Single;
    filter: TdtQueryFilter;
    options: Cardinal;
    raycastLimitSqr: Single;
  end;

  /// Provides the ability to perform pathfinding related queries against
  /// a navigation mesh.
  /// @ingroup detour
  TdtNavMeshQuery = class
  private
    m_nav: TdtNavMesh;          ///< Pointer to navmesh data.
    m_tinyNodePool:  TdtNodePool;///< Pointer to small node pool.
    m_nodePool: TdtNodePool;    ///< Pointer to node pool.
    m_openList:  TdtNodeQueue;    ///< Pointer to open list queue.
    m_query: TdtQueryData;      ///< Sliced query state.
  public
    constructor Create;
    destructor Destroy; override;

    /// Initializes the query object.
    ///  @param[in]    nav      Pointer to the dtNavMesh object to use for all queries.
    ///  @param[in]    maxNodes  Maximum number of search nodes. [Limits: 0 < value <= 65536]
    /// @returns The status flags for the query.
    function init(nav: TdtNavMesh; maxNodes: Integer): TdtStatus;

    /// @name Standard Pathfinding Functions
    // /@{

    /// Finds a path from the start polygon to the end polygon.
    ///  @param[in]    startRef  The refrence id of the start polygon.
    ///  @param[in]    endRef    The reference id of the end polygon.
    ///  @param[in]    startPos  A position within the start polygon. [(x, y, z)]
    ///  @param[in]    endPos    A position within the end polygon. [(x, y, z)]
    ///  @param[in]    filter    The polygon filter to apply to the query.
    ///  @param[out]  path    An ordered list of polygon references representing the path. (Start to end.)
    ///                [(polyRef) * @p pathCount]
    ///  @param[out]  pathCount  The number of polygons returned in the @p path array.
    ///  @param[in]    maxPath    The maximum number of polygons the @p path array can hold. [Limit: >= 1]
    function findPath(startRef, endRef: TdtPolyRef;
              startPos, endPos: PSingle;
              filter: TdtQueryFilter;
              path: PdtPolyRef; pathCount: PInteger; maxPath: Integer): TdtStatus;

    /// Finds the straight path from the start to the end position within the polygon corridor.
    ///  @param[in]    startPos      Path start position. [(x, y, z)]
    ///  @param[in]    endPos        Path end position. [(x, y, z)]
    ///  @param[in]    path        An array of polygon references that represent the path corridor.
    ///  @param[in]    pathSize      The number of polygons in the @p path array.
    ///  @param[out]  straightPath    Points describing the straight path. [(x, y, z) * @p straightPathCount].
    ///  @param[out]  straightPathFlags  Flags describing each point. (See: #dtStraightPathFlags) [opt]
    ///  @param[out]  straightPathRefs  The reference id of the polygon that is being entered at each point. [opt]
    ///  @param[out]  straightPathCount  The number of points in the straight path.
    ///  @param[in]    maxStraightPath    The maximum number of points the straight path arrays can hold.  [Limit: > 0]
    ///  @param[in]    options        Query options. (see: #dtStraightPathOptions)
    /// @returns The status flags for the query.
    function findStraightPath(startPos, endPos: PSingle;
                  path: PdtPolyRef; pathSize: Integer;
                  straightPath: PSingle; straightPathFlags: PByte; straightPathRefs: PdtPolyRef;
                  straightPathCount: PInteger; maxStraightPath: Integer; options: Integer = 0): TdtStatus;

    ///@}
    /// @name Sliced Pathfinding Functions
    /// Common use case:
    ///  -# Call initSlicedFindPath() to initialize the sliced path query.
    ///  -# Call updateSlicedFindPath() until it returns complete.
    ///  -# Call finalizeSlicedFindPath() to get the path.
    ///@{

    /// Intializes a sliced path query.
    ///  @param[in]    startRef  The refrence id of the start polygon.
    ///  @param[in]    endRef    The reference id of the end polygon.
    ///  @param[in]    startPos  A position within the start polygon. [(x, y, z)]
    ///  @param[in]    endPos    A position within the end polygon. [(x, y, z)]
    ///  @param[in]    filter    The polygon filter to apply to the query.
    ///  @param[in]    options    query options (see: #dtFindPathOptions)
    /// @returns The status flags for the query.
    function initSlicedFindPath(startRef, endRef: TdtPolyRef;
                  startPos, endPos: PSingle;
                  filter: TdtQueryFilter; options: Integer = 0): TdtStatus;

    /// Updates an in-progress sliced path query.
    ///  @param[in]    maxIter    The maximum number of iterations to perform.
    ///  @param[out]  doneIters  The actual number of iterations completed. [opt]
    /// @returns The status flags for the query.
    function updateSlicedFindPath(maxIter: Integer; doneIters: PInteger): TdtStatus;

    /// Finalizes and returns the results of a sliced path query.
    ///  @param[out]  path    An ordered list of polygon references representing the path. (Start to end.)
    ///                [(polyRef) * @p pathCount]
    ///  @param[out]  pathCount  The number of polygons returned in the @p path array.
    ///  @param[in]    maxPath    The max number of polygons the path array can hold. [Limit: >= 1]
    /// @returns The status flags for the query.
    function finalizeSlicedFindPath(path: PdtPolyRef; pathCount: PInteger; maxPath: Integer): TdtStatus;

    /// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
    /// polygon on the existing path that was visited during the search.
    ///  @param[in]    existing    An array of polygon references for the existing path.
    ///  @param[in]    existingSize  The number of polygon in the @p existing array.
    ///  @param[out]  path      An ordered list of polygon references representing the path. (Start to end.)
    ///                  [(polyRef) * @p pathCount]
    ///  @param[out]  pathCount    The number of polygons returned in the @p path array.
    ///  @param[in]    maxPath      The max number of polygons the @p path array can hold. [Limit: >= 1]
    /// @returns The status flags for the query.
    function finalizeSlicedFindPathPartial(existing: PdtPolyRef; existingSize: Integer;
                         path: PdtPolyRef; pathCount: PInteger; maxPath: Integer): TdtStatus;

    ///@}
    /// @name Dijkstra Search Functions
    /// @{

    /// Finds the polygons along the navigation graph that touch the specified circle.
    ///  @param[in]    startRef    The reference id of the polygon where the search starts.
    ///  @param[in]    centerPos    The center of the search circle. [(x, y, z)]
    ///  @param[in]    radius      The radius of the search circle.
    ///  @param[in]    filter      The polygon filter to apply to the query.
    ///  @param[out]  resultRef    The reference ids of the polygons touched by the circle. [opt]
    ///  @param[out]  resultParent  The reference ids of the parent polygons for each result.
    ///                  Zero if a result polygon has no parent. [opt]
    ///  @param[out]  resultCost    The search cost from @p centerPos to the polygon. [opt]
    ///  @param[out]  resultCount    The number of polygons found. [opt]
    ///  @param[in]    maxResult    The maximum number of polygons the result arrays can hold.
    /// @returns The status flags for the query.
    function findPolysAroundCircle(startRef: TdtPolyRef; centerPos: PSingle; radius: Single;
                     filter: TdtQueryFilter;
                     resultRef, resultParent: PdtPolyRef; resultCost: PSingle;
                     resultCount: PInteger; maxResult: Integer): TdtStatus;

    /// Finds the polygons along the naviation graph that touch the specified convex polygon.
    ///  @param[in]    startRef    The reference id of the polygon where the search starts.
    ///  @param[in]    verts      The vertices describing the convex polygon. (CCW)
    ///                  [(x, y, z) * @p nverts]
    ///  @param[in]    nverts      The number of vertices in the polygon.
    ///  @param[in]    filter      The polygon filter to apply to the query.
    ///  @param[out]  resultRef    The reference ids of the polygons touched by the search polygon. [opt]
    ///  @param[out]  resultParent  The reference ids of the parent polygons for each result. Zero if a
    ///                  result polygon has no parent. [opt]
    ///  @param[out]  resultCost    The search cost from the centroid point to the polygon. [opt]
    ///  @param[out]  resultCount    The number of polygons found.
    ///  @param[in]    maxResult    The maximum number of polygons the result arrays can hold.
    /// @returns The status flags for the query.
     function findPolysAroundShape(startRef: TdtPolyRef; verts: PSingle; nverts: Integer;
                    filter: TdtQueryFilter;
                    resultRef, resultParent: PdtPolyRef; resultCost: PSingle;
                    resultCount: PInteger; maxResult: Integer): TdtStatus;

    /// @}
    /// @name Local Query Functions
    ///@{

    /// Finds the polygon nearest to the specified center point.
    ///  @param[in]    center    The center of the search box. [(x, y, z)]
    ///  @param[in]    extents    The search distance along each axis. [(x, y, z)]
    ///  @param[in]    filter    The polygon filter to apply to the query.
    ///  @param[out]  nearestRef  The reference id of the nearest polygon.
    ///  @param[out]  nearestPt  The nearest point on the polygon. [opt] [(x, y, z)]
    /// @returns The status flags for the query.
    function findNearestPoly(center, extents: PSingle; filter: TdtQueryFilter; nearestRef: PdtPolyRef; nearestPt: PSingle): TdtStatus;

    /// Finds polygons that overlap the search box.
    ///  @param[in]    center    The center of the search box. [(x, y, z)]
    ///  @param[in]    extents    The search distance along each axis. [(x, y, z)]
    ///  @param[in]    filter    The polygon filter to apply to the query.
    ///  @param[out]  polys    The reference ids of the polygons that overlap the query box.
    ///  @param[out]  polyCount  The number of polygons in the search result.
    ///  @param[in]    maxPolys  The maximum number of polygons the search result can hold.
    /// @returns The status flags for the query.
    function queryPolygons(center, extents: PSingle; filter: TdtQueryFilter; polys: PdtPolyRef; polyCount: PInteger; maxPolys: Integer): TdtStatus;

    /// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
    ///  @param[in]    startRef    The reference id of the polygon where the search starts.
    ///  @param[in]    centerPos    The center of the query circle. [(x, y, z)]
    ///  @param[in]    radius      The radius of the query circle.
    ///  @param[in]    filter      The polygon filter to apply to the query.
    ///  @param[out]  resultRef    The reference ids of the polygons touched by the circle.
    ///  @param[out]  resultParent  The reference ids of the parent polygons for each result.
    ///                  Zero if a result polygon has no parent. [opt]
    ///  @param[out]  resultCount    The number of polygons found.
    ///  @param[in]    maxResult    The maximum number of polygons the result arrays can hold.
    /// @returns The status flags for the query.
    function findLocalNeighbourhood(startRef: TdtPolyRef; centerPos: PSingle; radius: Single;
                    filter: TdtQueryFilter;
                    resultRef, resultParent: PdtPolyRef;
                    resultCount: PInteger; maxResult: Integer): TdtStatus;

    /// Moves from the start to the end position constrained to the navigation mesh.
    ///  @param[in]    startRef    The reference id of the start polygon.
    ///  @param[in]    startPos    A position of the mover within the start polygon. [(x, y, x)]
    ///  @param[in]    endPos      The desired end position of the mover. [(x, y, z)]
    ///  @param[in]    filter      The polygon filter to apply to the query.
    ///  @param[out]  resultPos    The result position of the mover. [(x, y, z)]
    ///  @param[out]  visited      The reference ids of the polygons visited during the move.
    ///  @param[out]  visitedCount  The number of polygons visited during the move.
    ///  @param[in]    maxVisitedSize  The maximum number of polygons the @p visited array can hold.
    /// @returns The status flags for the query.
    function moveAlongSurface(startRef: TdtPolyRef; startPos, endPos: PSingle;
                  filter: TdtQueryFilter;
                  resultPos: PSingle; visited: PdtPolyRef; visitedCount: PInteger; maxVisitedSize: Integer): TdtStatus;

    /// Casts a 'walkability' ray along the surface of the navigation mesh from
    /// the start position toward the end position.
    /// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
    ///  @param[in]    startRef  The reference id of the start polygon.
    ///  @param[in]    startPos  A position within the start polygon representing
    ///                the start of the ray. [(x, y, z)]
    ///  @param[in]    endPos    The position to cast the ray toward. [(x, y, z)]
    ///  @param[out]  t      The hit parameter. (FLT_MAX if no wall hit.)
    ///  @param[out]  hitNormal  The normal of the nearest wall hit. [(x, y, z)]
    ///  @param[in]    filter    The polygon filter to apply to the query.
    ///  @param[out]  path    The reference ids of the visited polygons. [opt]
    ///  @param[out]  pathCount  The number of visited polygons. [opt]
    ///  @param[in]    maxPath    The maximum number of polygons the @p path array can hold.
    /// @returns The status flags for the query.
    function raycast(startRef: TdtPolyRef; startPos, endPos: PSingle;
             filter: TdtQueryFilter;
             t, hitNormal: PSingle; path: PdtPolyRef; pathCount: PInteger; maxPath: Integer): TdtStatus; overload;

    /// Casts a 'walkability' ray along the surface of the navigation mesh from
    /// the start position toward the end position.
    ///  @param[in]    startRef  The reference id of the start polygon.
    ///  @param[in]    startPos  A position within the start polygon representing
    ///                the start of the ray. [(x, y, z)]
    ///  @param[in]    endPos    The position to cast the ray toward. [(x, y, z)]
    ///  @param[in]    filter    The polygon filter to apply to the query.
    ///  @param[in]    flags    govern how the raycast behaves. See dtRaycastOptions
    ///  @param[out]  hit      Pointer to a raycast hit structure which will be filled by the results.
    ///  @param[in]    prevRef    parent of start ref. Used during for cost calculation [opt]
    /// @returns The status flags for the query.
    function raycast(startRef: TdtPolyRef; startPos, endPos: PSingle;
             filter: TdtQueryFilter; options: Cardinal;
             hit: PdtRaycastHit; prevRef: TdtPolyRef = 0): TdtStatus; overload;


    /// Finds the distance from the specified position to the nearest polygon wall.
    ///  @param[in]    startRef    The reference id of the polygon containing @p centerPos.
    ///  @param[in]    centerPos    The center of the search circle. [(x, y, z)]
    ///  @param[in]    maxRadius    The radius of the search circle.
    ///  @param[in]    filter      The polygon filter to apply to the query.
    ///  @param[out]  hitDist      The distance to the nearest wall from @p centerPos.
    ///  @param[out]  hitPos      The nearest position on the wall that was hit. [(x, y, z)]
    ///  @param[out]  hitNormal    The normalized ray formed from the wall point to the
    ///                  source point. [(x, y, z)]
    /// @returns The status flags for the query.
    function findDistanceToWall(startRef: TdtPolyRef; centerPos: PSingle; maxRadius: Single;
                  filter: TdtQueryFilter;
                  hitDist, hitPos, hitNormal: PSingle): TdtStatus;

    /// Returns the segments for the specified polygon, optionally including portals.
    ///  @param[in]    ref        The reference id of the polygon.
    ///  @param[in]    filter      The polygon filter to apply to the query.
    ///  @param[out]  segmentVerts  The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
    ///  @param[out]  segmentRefs    The reference ids of each segment's neighbor polygon.
    ///                  Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount]
    ///  @param[out]  segmentCount  The number of segments returned.
    ///  @param[in]    maxSegments    The maximum number of segments the result arrays can hold.
    /// @returns The status flags for the query.
    function getPolyWallSegments(ref: TdtPolyRef; filter: TdtQueryFilter;
                   segmentVerts: PSingle; segmentRefs: PdtPolyRef; segmentCount: PInteger;
                   maxSegments: Integer): TdtStatus;

    /// Returns random location on navmesh.
    /// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
    ///  @param[in]    filter      The polygon filter to apply to the query.
    ///  @param[in]    frand      Function returning a random number [0..1).
    ///  @param[out]  randomRef    The reference id of the random location.
    ///  @param[out]  randomPt    The random location.
    /// @returns The status flags for the query.
    function findRandomPoint(filter: TdtQueryFilter; //float ( *frand)(),
                 randomRef: PdtPolyRef; randomPt: PSingle): TdtStatus;

    /// Returns random location on navmesh within the reach of specified location.
    /// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
    /// The location is not exactly constrained by the circle, but it limits the visited polygons.
    ///  @param[in]    startRef    The reference id of the polygon where the search starts.
    ///  @param[in]    centerPos    The center of the search circle. [(x, y, z)]
    ///  @param[in]    filter      The polygon filter to apply to the query.
    ///  @param[in]    frand      Function returning a random number [0..1).
    ///  @param[out]  randomRef    The reference id of the random location.
    ///  @param[out]  randomPt    The random location. [(x, y, z)]
    /// @returns The status flags for the query.
    function findRandomPointAroundCircle(startRef: TdtPolyRef; centerPos: PSingle; maxRadius: Single;
                       filter: TdtQueryFilter; //{float ( *frand)(),
                       randomRef: PdtPolyRef; randomPt: PSingle): TdtStatus;

    /// Finds the closest point on the specified polygon.
    ///  @param[in]    ref      The reference id of the polygon.
    ///  @param[in]    pos      The position to check. [(x, y, z)]
    ///  @param[out]  closest    The closest point on the polygon. [(x, y, z)]
    ///  @param[out]  posOverPoly  True of the position is over the polygon.
    /// @returns The status flags for the query.
    function closestPointOnPoly(ref: TdtPolyRef; pos, closest: PSingle; posOverPoly: PBoolean): TdtStatus;

    /// Returns a point on the boundary closest to the source point if the source point is outside the
    /// polygon's xz-bounds.
    ///  @param[in]    ref      The reference id to the polygon.
    ///  @param[in]    pos      The position to check. [(x, y, z)]
    ///  @param[out]  closest    The closest point. [(x, y, z)]
    /// @returns The status flags for the query.
    function closestPointOnPolyBoundary(ref: TdtPolyRef; pos, closest: PSingle): TdtStatus;

    /// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
    ///  @param[in]    ref      The reference id of the polygon.
    ///  @param[in]    pos      A position within the xz-bounds of the polygon. [(x, y, z)]
    ///  @param[out]  height    The height at the surface of the polygon.
    /// @returns The status flags for the query.
    function getPolyHeight(ref: TdtPolyRef; pos, height: PSingle): TdtStatus;

    /// @}
    /// @name Miscellaneous Functions
    /// @{

    /// Returns true if the polygon reference is valid and passes the filter restrictions.
    ///  @param[in]    ref      The polygon reference to check.
    ///  @param[in]    filter    The filter to apply.
    function isValidPolyRef(ref: TdtPolyRef; filter: TdtQueryFilter): Boolean;

    /// Returns true if the polygon reference is in the closed list.
    ///  @param[in]    ref    The reference id of the polygon to check.
    /// @returns True if the polygon is in closed list.
    function isInClosedList(ref: TdtPolyRef): Boolean;

    /// Gets the node pool.
    /// @returns The node pool.
    property getNodePool: TdtNodePool read m_nodePool;

    /// Gets the navigation mesh the query object is using.
    /// @return The navigation mesh the query object is using.
    property getAttachedNavMesh: TdtNavMesh read m_nav;

    /// @}
  private

    /// Returns neighbour tile based on side.
  {  function getNeighbourTileAt(x, y, side: Integer): PdtMeshTile;}

    /// Queries polygons within a tile.
    function queryPolygonsInTile(tile: PdtMeshTile; qmin, qmax: PSingle; filter: TdtQueryFilter;
                polys: PdtPolyRef; maxPolys: Integer): Integer;

    /// Returns portal points between two polygons.
    function getPortalPoints(from, &to: TdtPolyRef; left, right: PSingle;
                 fromType, toType: PByte): TdtStatus; overload;
    function getPortalPoints(from: TdtPolyRef; fromPoly: PdtPoly; fromTile: PdtMeshTile;
                 &to: TdtPolyRef; toPoly: PdtPoly; toTile: PdtMeshTile;
                 left, right: PSingle): TdtStatus; overload;

    /// Returns edge mid point between two polygons.
    function getEdgeMidPoint(from, &to: TdtPolyRef; mid: PSingle): TdtStatus; overload;
    function getEdgeMidPoint(from: TdtPolyRef; fromPoly: PdtPoly; fromTile: PdtMeshTile;
                 &to: TdtPolyRef; toPoly: PdtPoly; toTile: PdtMeshTile;
                 mid: PSingle): TdtStatus; overload;

    // Appends vertex to a straight path
    function appendVertex(pos: PSingle; flags: Integer; ref: TdtPolyRef;
                straightPath: PSingle; straightPathFlags: PByte; straightPathRefs: PdtPolyRef;
                straightPathCount: PInteger; maxStraightPath: Integer): TdtStatus;

    // Appends intermediate portal points to a straight path.
    function appendPortals(startIdx, endIdx: Integer; endPos: PSingle; path: PdtPolyRef;
                 straightPath: PSingle; straightPathFlags: PByte; straightPathRefs: PdtPolyRef;
                 straightPathCount: PInteger; maxStraightPath, options: Integer): TdtStatus;
  end;

  /// Allocates a query object using the Detour allocator.
  /// @return An allocated query object, or null on failure.
  /// @ingroup detour
  function dtAllocNavMeshQuery(): TdtNavMeshQuery;

  /// Frees the specified query object using the Detour allocator.
  ///  @param[in]    query    A query object allocated using #dtAllocNavMeshQuery
  /// @ingroup detour
  procedure dtFreeNavMeshQuery(var navmesh: TdtNavMeshQuery);

implementation
uses Math;

/// @class dtQueryFilter
///
/// <b>The Default Implementation</b>
///
/// At construction: All area costs default to 1.0.  All flags are included
/// and none are excluded.
///
/// If a polygon has both an include and an exclude flag, it will be excluded.
///
/// The way filtering works, a navigation mesh polygon must have at least one flag
/// set to ever be considered by a query. So a polygon with no flags will never
/// be considered.
///
/// Setting the include flags to 0 will result in all polygons being excluded.
///
/// <b>Custom Implementations</b>
///
/// DT_VIRTUAL_QUERYFILTER must be defined in order to extend this class.
///
/// Implement a custom query filter by overriding the virtual passFilter()
/// and getCost() functions. If this is done, both functions should be as
/// fast as possible. Use cached local copies of data rather than accessing
/// your own objects where possible.
///
/// Custom implementations do not need to adhere to the flags or cost logic
/// used by the default implementation.
///
/// In order for A* searches to work properly, the cost should be proportional to
/// the travel distance. Implementing a cost modifier less than 1.0 is likely
/// to lead to problems during pathfinding.
///
/// @see dtNavMeshQuery

constructor TdtQueryFilter.Create;
var i: Integer;
begin
  m_includeFlags := $ffff;
  m_excludeFlags := 0;

  for i := 0 to DT_MAX_AREAS - 1 do
    m_areaCost[i] := 1.0;
end;

function TdtQueryFilter.passFilter(ref: TdtPolyRef; tile: PdtMeshTile; poly: PdtPoly): Boolean;
begin
  Result := ((poly.flags and m_includeFlags) <> 0) and ((poly.flags and m_excludeFlags) = 0);
end;

function TdtQueryFilter.getCost(pa, pb: PSingle;
          prevRef: TdtPolyRef; prevTile: PdtMeshTile; prevPoly: PdtPoly;
          curRef: TdtPolyRef; curTile: PdtMeshTile; curPoly: PdtPoly;
          nextRef: TdtPolyRef; nextTile: PdtMeshTile; nextPoly: PdtPoly): Single;
begin
  Result := dtVdist(pa, pb) * m_areaCost[curPoly.getArea()];
end;

function TdtQueryFilter.getAreaCost(i: Integer): Single; begin Result := m_areaCost[i]; end;
procedure TdtQueryFilter.setAreaCost(i: Integer; cost: Single); begin m_areaCost[i] := cost; end;
function TdtQueryFilter.getIncludeFlags(): Word; begin Result := m_includeFlags; end;
procedure TdtQueryFilter.setIncludeFlags(flags: Word); begin m_includeFlags := flags; end;
function TdtQueryFilter.getExcludeFlags(): Word; begin Result := m_excludeFlags; end;
procedure TdtQueryFilter.setExcludeFlags(flags: Word); begin m_excludeFlags := flags; end;

const H_SCALE = 0.999; // Search heuristic scale.

function dtAllocNavMeshQuery(): TdtNavMeshQuery;
begin
  Result := TdtNavMeshQuery.Create;
end;

procedure dtFreeNavMeshQuery(var navmesh: TdtNavMeshQuery);
begin
  if (navmesh = nil) then Exit;
  navMesh.Free;
  navMesh := nil;
end;

//////////////////////////////////////////////////////////////////////////////////////////

/// @class dtNavMeshQuery
///
/// For methods that support undersized buffers, if the buffer is too small
/// to hold the entire result set the return status of the method will include
/// the #DT_BUFFER_TOO_SMALL flag.
///
/// Constant member functions can be used by multiple clients without side
/// effects. (E.g. No change to the closed list. No impact on an in-progress
/// sliced path query. Etc.)
///
/// Walls and portals: A @e wall is a polygon segment that is
/// considered impassable. A @e portal is a passable segment between polygons.
/// A portal may be treated as a wall based on the dtQueryFilter used for a query.
///
/// @see dtNavMesh, dtQueryFilter, #dtAllocNavMeshQuery(), #dtAllocNavMeshQuery()

constructor TdtNavMeshQuery.Create;
begin
  inherited;

end;

destructor TdtNavMeshQuery.Destroy;
begin
  m_tinyNodePool.Free;
  m_nodePool.Free;
  m_openList.Free;

  inherited;
end;

/// @par
///
/// Must be the first function called after construction, before other
/// functions are used.
///
/// This function can be used multiple times.
function TdtNavMeshQuery.init(nav: TdtNavMesh; maxNodes: Integer): TdtStatus;
begin
  m_nav := nav;

  if (m_nodePool = nil) or (m_nodePool.getMaxNodes < maxNodes) then
  begin
    if (m_nodePool <> nil) then
    begin
      m_nodePool.Free;
      m_nodePool := nil;
    end;
    m_nodePool := TdtNodePool.Create(maxNodes, dtNextPow2(maxNodes div 4));
    if (m_nodePool = nil) then
      Exit(DT_FAILURE or DT_OUT_OF_MEMORY);
  end
  else
  begin
    m_nodePool.clear();
  end;

  if (m_tinyNodePool = nil) then
  begin
    m_tinyNodePool := TdtNodePool.Create(64, 32);
    if (m_tinyNodePool = nil) then
      Exit(DT_FAILURE or DT_OUT_OF_MEMORY);
  end
  else
  begin
    m_tinyNodePool.clear();
  end;

  // TODO: check the open list size too.
  if (m_openList = nil) or (m_openList.getCapacity < maxNodes) then
  begin
    if (m_openList <> nil) then
    begin
      m_openList.Free;
      m_openList := nil;
    end;
    m_openList := TdtNodeQueue.Create(maxNodes);
    if (m_openList = nil) then
      Exit(DT_FAILURE or DT_OUT_OF_MEMORY);
  end
  else
  begin
    m_openList.clear();
  end;

  Result := DT_SUCCESS;
end;

function TdtNavMeshQuery.findRandomPoint(filter: TdtQueryFilter; //float ( *frand)(),
               randomRef: PdtPolyRef; randomPt: PSingle): TdtStatus;
var tile: PdtMeshTile; tsum,area,u,areaSum,polyArea,rs,rt,h: Single; i,j: Integer; t: PdtMeshTile; poly,p: PdtPoly;
polyRef,base,ref: TdtPolyRef; va,vb,vc,v: PSingle; verts: array [0..3*DT_VERTS_PER_POLYGON-1] of Single;
areas: array [0..DT_VERTS_PER_POLYGON-1] of Single; pt: array [0..2] of Single; status: TdtStatus;
begin
  Assert(m_nav <> nil);

  // Randomly pick one tile. Assume that all tiles cover roughly the same area.
  tile := nil;
  tsum := 0.0;
  for i := 0 to m_nav.getMaxTiles - 1 do
  begin
    t := m_nav.getTile(i);
    if (t = nil) or(t.header = nil) then continue;

    // Choose random tile using reservoi sampling.
    area := 1.0; // Could be tile area too.
    tsum := tsum + area;
    u := Random();
    if (u*tsum <= area) then
      tile := t;
  end;
  if (tile = nil) then
    Exit(DT_FAILURE);

  // Randomly pick one polygon weighted by polygon area.
  poly := nil;
  polyRef := 0;
  base := m_nav.getPolyRefBase(tile);

  areaSum := 0.0;
  for i := 0 to tile.header.polyCount - 1 do
  begin
    p := @tile.polys[i];
    // Do not return off-mesh connection polygons.
    if (p.getType() <> DT_POLYTYPE_GROUND) then
      continue;
    // Must pass filter
    ref := base or TdtPolyRef(i);
    if (not filter.passFilter(ref, tile, p)) then
      continue;

    // Calc area of the polygon.
    polyArea := 0.0;
    for j := 2 to p.vertCount - 1 do
    begin
      va := @tile.verts[p.verts[0]*3];
      vb := @tile.verts[p.verts[j-1]*3];
      vc := @tile.verts[p.verts[j]*3];
      polyArea := polyArea + dtTriArea2D(va,vb,vc);
    end;

    // Choose random polygon weighted by area, using reservoi sampling.
    areaSum := areaSum + polyArea;
    u := Random();
    if (u*areaSum <= polyArea) then
    begin
      poly := p;
      polyRef := ref;
    end;
  end;

  if (poly = nil) then
    Exit(DT_FAILURE);

  // Randomly pick point on polygon.
  v := @tile.verts[poly.verts[0]*3];
  dtVcopy(@verts[0*3],v);
  for j := 1 to poly.vertCount - 1 do
  begin
    v := @tile.verts[poly.verts[j]*3];
    dtVcopy(@verts[j*3],v);
  end;

  rs := Random();
  rt := Random();

  dtRandomPointInConvexPoly(@verts[0], poly.vertCount, @areas[0], rs, rt, @pt[0]);

  h := 0.0;
  status := getPolyHeight(polyRef, @pt[0], @h);
  if (dtStatusFailed(status)) then
    Exit(status);
  pt[1] := h;

  dtVcopy(randomPt, @pt[0]);
  randomRef^ := polyRef;

  Result := DT_SUCCESS;
end;

function TdtNavMeshQuery.findRandomPointAroundCircle(startRef: TdtPolyRef; centerPos: PSingle; maxRadius: Single;
                     filter: TdtQueryFilter; //float ( *frand)(),
                     randomRef: PdtPolyRef; randomPt: PSingle): TdtStatus;
var startTile,randomTile,bestTile,parentTile,neighbourTile: PdtMeshTile; startPoly,randomPoly,bestPoly,parentPoly,neighbourPoly: PdtPoly;
startNode,bestNode,neighbourNode: PdtNode; status: TdtStatus; radiusSqr,areaSum: Single; randomPolyRef,bestRef,parentRef,neighbourRef: TdtPolyRef;
polyArea,u,s,t,h,tseg,distSqr,total: Single; j: Integer; va,vb,vc,v: PSingle; i: Cardinal; link: PdtLink; va3,vb3,pt: array [0..2] of Single;
verts: array [0..3*DT_VERTS_PER_POLYGON-1] of Single; areas: array [0..DT_VERTS_PER_POLYGON-1] of Single; stat: TdtStatus;
begin
  Assert(m_nav <> nil);
  Assert(m_nodePool <> nil);
  Assert(m_openList <> nil);

  // Validate input
  if (startRef = 0) or (not m_nav.isValidPolyRef(startRef)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  startTile := nil;
  startPoly := nil;
  m_nav.getTileAndPolyByRefUnsafe(startRef, @startTile, @startPoly);
  if (not filter.passFilter(startRef, startTile, startPoly)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  m_nodePool.clear();
  m_openList.clear();

  startNode := m_nodePool.getNode(startRef);
  dtVcopy(@startNode.pos, centerPos);
  startNode.pidx := 0;
  startNode.cost := 0;
  startNode.total := 0;
  startNode.id := startRef;
  startNode.flags := DT_NODE_OPEN;
  m_openList.push(startNode);

  status := DT_SUCCESS;

  radiusSqr := Sqr(maxRadius);
  areaSum := 0.0;

  randomTile := nil;
  randomPoly := nil;
  randomPolyRef := 0;

  while (not m_openList.empty()) do
  begin
    bestNode := m_openList.pop();
    bestNode.flags := bestNode.flags and not DT_NODE_OPEN;
    bestNode.flags := bestNode.flags or DT_NODE_CLOSED;

    // Get poly and tile.
    // The API input has been cheked already, skip checking internal data.
    bestRef := bestNode.id;
    bestTile := nil;
    bestPoly := nil;
    m_nav.getTileAndPolyByRefUnsafe(bestRef, @bestTile, @bestPoly);

    // Place random locations on on ground.
    if (bestPoly.getType() = DT_POLYTYPE_GROUND) then
    begin
      // Calc area of the polygon.
      polyArea := 0.0;
      for j := 2 to bestPoly.vertCount - 1 do
      begin
        va := @bestTile.verts[bestPoly.verts[0]*3];
        vb := @bestTile.verts[bestPoly.verts[j-1]*3];
        vc := @bestTile.verts[bestPoly.verts[j]*3];
        polyArea := polyArea + dtTriArea2D(va,vb,vc);
      end;
      // Choose random polygon weighted by area, using reservoi sampling.
      areaSum := areaSum + polyArea;
      u := Random;
      if (u*areaSum <= polyArea) then
      begin
        randomTile := bestTile;
        randomPoly := bestPoly;
        randomPolyRef := bestRef;
      end;
    end;


    // Get parent poly and tile.
    parentRef := 0;
    parentTile := nil;
    parentPoly := nil;
    if (bestNode.pidx <> 0) then
      parentRef := m_nodePool.getNodeAtIdx(bestNode.pidx).id;
    if (parentRef <> 0) then
      m_nav.getTileAndPolyByRefUnsafe(parentRef, @parentTile, @parentPoly);

    i := bestPoly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      link := @bestTile.links[i];
      neighbourRef := link.ref;
      // Skip invalid neighbours and do not follow back to parent.
      if (neighbourRef = 0) or (neighbourRef = parentRef) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Expand to neighbour
      neighbourTile := nil;
      neighbourPoly := nil;
      m_nav.getTileAndPolyByRefUnsafe(neighbourRef, @neighbourTile, @neighbourPoly);

      // Do not advance if the polygon is excluded by the filter.
      if (not filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Find edge and calc distance to the edge.
      if (getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, @va3[0], @vb3[0]) = 0) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // If the circle is not touching the next polygon, skip it.
      distSqr := dtDistancePtSegSqr2D(centerPos, @va3[0], @vb3[0], @tseg);
      if (distSqr > radiusSqr) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      neighbourNode := m_nodePool.getNode(neighbourRef);
      if (neighbourNode = nil) then
      begin
        status := status or DT_OUT_OF_NODES;
        i := bestTile.links[i].next;
        continue;
      end;

      if (neighbourNode.flags and DT_NODE_CLOSED) <> 0 then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Cost
      if (neighbourNode.flags = 0) then
        dtVlerp(@neighbourNode.pos, @va3[0], @vb3[0], 0.5);

      total := bestNode.total + dtVdist(@bestNode.pos, @neighbourNode.pos);

      // The node is already in open list and the new result is worse, skip.
      if ((neighbourNode.flags and DT_NODE_OPEN) <> 0) and (total >= neighbourNode.total) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      neighbourNode.id := neighbourRef;
      neighbourNode.flags := (neighbourNode.flags and not DT_NODE_CLOSED);
      neighbourNode.pidx := m_nodePool.getNodeIdx(bestNode);
      neighbourNode.total := total;

      if (neighbourNode.flags and DT_NODE_OPEN) <> 0 then
      begin
        m_openList.modify(neighbourNode);
      end
      else
      begin
        neighbourNode.flags := DT_NODE_OPEN;
        m_openList.push(neighbourNode);
      end;

      i := bestTile.links[i].next;
    end;
  end;

  if (randomPoly = nil) then
    Exit(DT_FAILURE);

  // Randomly pick point on polygon.
  v := @randomTile.verts[randomPoly.verts[0]*3];
  dtVcopy(@verts[0*3],v);
  for j := 1 to randomPoly.vertCount - 1 do
  begin
    v := @randomTile.verts[randomPoly.verts[j]*3];
    dtVcopy(@verts[j*3],v);
  end;

  s := Random;
  t := Random;

  dtRandomPointInConvexPoly(@verts[0], randomPoly.vertCount, @areas[0], s, t, @pt[0]);

  h := 0.0;
  stat := getPolyHeight(randomPolyRef, @pt[0], @h);
  if (dtStatusFailed(status)) then
    Exit(stat);
  pt[1] := h;

  dtVcopy(randomPt, @pt[0]);
  randomRef^ := randomPolyRef;

  Result := DT_SUCCESS;
end;


//////////////////////////////////////////////////////////////////////////////////////////

/// @par
///
/// Uses the detail polygons to find the surface height. (Most accurate.)
///
/// @p pos does not have to be within the bounds of the polygon or navigation mesh.
///
/// See closestPointOnPolyBoundary() for a limited but faster option.
///
function TdtNavMeshQuery.closestPointOnPoly(ref: TdtPolyRef; pos, closest: PSingle; posOverPoly: PBoolean): TdtStatus;
var tile: PdtMeshTile; poly: PdtPoly; v0,v1: PSingle; d0,d1,u: Single; ip: Cardinal; pd: PdtPolyDetail;
verts: array [0..DT_VERTS_PER_POLYGON*3-1] of Single; edged: array [0..DT_VERTS_PER_POLYGON-1] of Single;
edget: array [0..DT_VERTS_PER_POLYGON-1] of Single; nv,i,j,k: Integer; dmin: Single; imin: Integer; va,vb: PSingle; t: PByte;
v: array [0..2] of PSingle; h: Single;
begin
  //dtAssert(m_nav);
  tile := nil;
  poly := nil;
  if (dtStatusFailed(m_nav.getTileAndPolyByRef(ref, @tile, @poly))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (tile = nil) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // Off-mesh connections don't have detail polygons.
  if (poly.getType() = DT_POLYTYPE_OFFMESH_CONNECTION) then
  begin
    v0 := @tile.verts[poly.verts[0]*3];
    v1 := @tile.verts[poly.verts[1]*3];
    d0 := dtVdist(pos, v0);
    d1 := dtVdist(pos, v1);
    u := d0 / (d0+d1);
    dtVlerp(closest, v0, v1, u);
    if (posOverPoly <> nil) then
      posOverPoly^ := false;
    Exit(DT_SUCCESS);
  end;

  ip := Cardinal(poly - tile.polys);
  pd := @tile.detailMeshes[ip];

  // Clamp point to be inside the polygon.
  nv := poly.vertCount;
  for i := 0 to nv - 1 do
    dtVcopy(@verts[i*3], @tile.verts[poly.verts[i]*3]);

  dtVcopy(closest, pos);
  if (not dtDistancePtPolyEdgesSqr(pos, @verts[0], nv, @edged[0], @edget[0])) then
  begin
    // Point is outside the polygon, dtClamp to nearest edge.
    dmin := MaxSingle;
    imin := -1;
    for i := 0 to nv - 1 do
    begin
      if (edged[i] < dmin) then
      begin
        dmin := edged[i];
        imin := i;
      end;
    end;
    va := @verts[imin*3];
    vb := @verts[((imin+1) mod nv)*3];
    dtVlerp(closest, va, vb, edget[imin]);

    if (posOverPoly <> nil) then
      posOverPoly^ := false;
  end
  else
  begin
    if (posOverPoly <> nil) then
      posOverPoly^ := true;
  end;

  // Find height at the location.
  for j := 0 to pd.triCount - 1 do
  begin
    t := @tile.detailTris[(pd.triBase+j)*4];
    for k := 0 to 2 do
    begin
      if (t[k] < poly.vertCount) then
        v[k] := @tile.verts[poly.verts[t[k]]*3]
      else
        v[k] := @tile.detailVerts[(pd.vertBase+(t[k]-poly.vertCount))*3];
    end;

    if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], @h)) then
    begin
      closest[1] := h;
      break;
    end;
  end;

  Result := DT_SUCCESS;
end;

/// @par
///
/// Much faster than closestPointOnPoly().
///
/// If the provided position lies within the polygon's xz-bounds (above or below),
/// then @p pos and @p closest will be equal.
///
/// The height of @p closest will be the polygon boundary.  The height detail is not used.
///
/// @p pos does not have to be within the bounds of the polybon or the navigation mesh.
///
function TdtNavMeshQuery.closestPointOnPolyBoundary(ref: TdtPolyRef; pos, closest: PSingle): TdtStatus;
var tile: PdtMeshTile; poly: PdtPoly; verts: array [0..DT_VERTS_PER_POLYGON*3-1] of Single;
edged: array [0..DT_VERTS_PER_POLYGON-1] of Single; edget: array [0..DT_VERTS_PER_POLYGON-1] of Single; nv,i: Integer;
inside: Boolean; dmin: Single; imin: Integer; va,vb: PSingle;
begin
  //dtAssert(m_nav);

  tile := nil;
  poly := nil;
  if (dtStatusFailed(m_nav.getTileAndPolyByRef(ref, @tile, @poly))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // Collect vertices.
  nv := 0;
  for i := 0 to poly.vertCount - 1 do
  begin
    dtVcopy(@verts[nv*3], @tile.verts[poly.verts[i]*3]);
    Inc(nv);
  end;

  inside := dtDistancePtPolyEdgesSqr(pos, @verts[0], nv, @edged[0], @edget[0]);
  if (inside) then
  begin
    // Point is inside the polygon, return the point.
    dtVcopy(closest, pos);
  end
  else
  begin
    // Point is outside the polygon, dtClamp to nearest edge.
    dmin := MaxSingle;
    imin := -1;
    for i := 0 to nv - 1 do
    begin
      if (edged[i] < dmin) then
      begin
        dmin := edged[i];
        imin := i;
      end;
    end;
    va := @verts[imin*3];
    vb := @verts[((imin+1) mod nv)*3];
    dtVlerp(closest, va, vb, edget[imin]);
  end;

  Result := DT_SUCCESS;
end;

/// @par
///
/// Will return #DT_FAILURE if the provided position is outside the xz-bounds
/// of the polygon.
///
function TdtNavMeshQuery.getPolyHeight(ref: TdtPolyRef; pos, height: PSingle): TdtStatus;
var tile: PdtMeshTile; poly: PdtPoly; v0,v1: PSingle; d0,d1,u,h: Single; ip: Cardinal; pd: PdtPolyDetail; j,k: Integer; t: PByte;
v: array [0..2] of PSingle;
begin
  //dtAssert(m_nav);

  tile := nil;
  poly := nil;
  if (dtStatusFailed(m_nav.getTileAndPolyByRef(ref, @tile, @poly))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  if (poly.getType() = DT_POLYTYPE_OFFMESH_CONNECTION) then
  begin
    v0 := @tile.verts[poly.verts[0]*3];
    v1 := @tile.verts[poly.verts[1]*3];
    d0 := dtVdist2D(pos, v0);
    d1 := dtVdist2D(pos, v1);
    u := d0 / (d0+d1);
    if (height <> nil) then
      height^ := v0[1] + (v1[1] - v0[1]) * u;
    Exit(DT_SUCCESS);
  end
  else
  begin
    ip := (poly - tile.polys);
    pd := @tile.detailMeshes[ip];
    for j := 0 to pd.triCount - 1 do
    begin
      t := @tile.detailTris[(pd.triBase+j)*4];
      for k := 0 to 2 do
      begin
        if (t[k] < poly.vertCount) then
          v[k] := @tile.verts[poly.verts[t[k]]*3]
        else
          v[k] := @tile.detailVerts[(pd.vertBase+(t[k]-poly.vertCount))*3];
      end;
      if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], @h)) then
      begin
        if (height <> nil) then
          height^ := h;
        Exit(DT_SUCCESS);
      end;
    end;
  end;

  Result := (DT_FAILURE or DT_INVALID_PARAM);
end;

/// @par
///
/// @note If the search box does not intersect any polygons the search will
/// return #DT_SUCCESS, but @p nearestRef will be zero. So if in doubt, check
/// @p nearestRef before using @p nearestPt.
///
/// @warning This function is not suitable for large area searches.  If the search
/// extents overlaps more than MAX_SEARCH (128) polygons it may return an invalid result.
///
function TdtNavMeshQuery.findNearestPoly(center, extents: PSingle; filter: TdtQueryFilter; nearestRef: PdtPolyRef; nearestPt: PSingle): TdtStatus;
const MAX_SEARCH = 128;
var polys: array [0..MAX_SEARCH-1] of TdtPolyRef; polyCount: Integer; nearest,ref: TdtPolyRef; nearestDistanceSqr: Single; i: Integer;
closestPtPoly,diff: array [0..2] of Single; posOverPoly: Boolean; d: Single; tile: PdtMeshTile; poly: PdtPoly;
begin
  //dtAssert(m_nav);

  nearestRef^ := 0;

  // Get nearby polygons from proximity grid.
  polyCount := 0;
  if (dtStatusFailed(queryPolygons(center, extents, filter, @polys[0], @polyCount, MAX_SEARCH))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // Find nearest polygon amongst the nearby polygons.
  nearest := 0;
  nearestDistanceSqr := MaxSingle;
  for i := 0 to polyCount - 1 do
  begin
    ref := polys[i];
    posOverPoly := false;
    d := 0;
    closestPointOnPoly(ref, center, @closestPtPoly[0], @posOverPoly);

    // If a point is directly over a polygon and closer than
    // climb height, favor that instead of straight line nearest point.
    dtVsub(@diff[0], center, @closestPtPoly[0]);
    if (posOverPoly) then
    begin
      tile := nil;
      poly := nil;
      m_nav.getTileAndPolyByRefUnsafe(polys[i], @tile, @poly);
      d := Abs(diff[1]) - tile.header.walkableClimb;
      d := IfThen(d > 0, d*d, 0);
    end
    else
    begin
      d := dtVlenSqr(@diff[0]);
    end;

    if (d < nearestDistanceSqr) then
    begin
      if (nearestPt <> nil) then
        dtVcopy(nearestPt, @closestPtPoly[0]);
      nearestDistanceSqr := d;
      nearest := ref;
    end;
  end;

  if (nearestRef <> nil) then
    nearestRef^ := nearest;

  Result := DT_SUCCESS;
end;

function TdtNavMeshQuery.queryPolygonsInTile(tile: PdtMeshTile; qmin, qmax: PSingle; filter: TdtQueryFilter;
              polys: PdtPolyRef; maxPolys: Integer): Integer;
var node: PdtBVNode; &end: PdtBVNode; tbmin,tbmax: PSingle; qfac: Single; bmin,bmax: array [0..2] of Word; bminf,bmaxf: array [0..2] of Single;
minx,miny,minz,maxx,maxy,maxz: Single; base,ref: TdtPolyRef; i,j,n,escapeIndex: Integer; overlap,isLeafNode: Boolean; p: PdtPoly;
v: PSingle;
begin
  //dtAssert(m_nav);

  if (tile.bvTree <> nil) then
  begin
    node := @tile.bvTree[0];
    &end := @tile.bvTree[tile.header.bvNodeCount];
    tbmin := @tile.header.bmin[0];
    tbmax := @tile.header.bmax[0];
    qfac := tile.header.bvQuantFactor;

    // Calculate quantized box
    // dtClamp query box to world box.
    minx := dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
    miny := dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
    minz := dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
    maxx := dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
    maxy := dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
    maxz := dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
    // Quantize
    bmin[0] := Word(Trunc(qfac * minx)) and $fffe;
    bmin[1] := Word(Trunc(qfac * miny)) and $fffe;
    bmin[2] := Word(Trunc(qfac * minz)) and $fffe;
    bmax[0] := Word(Trunc(qfac * maxx + 1)) or 1;
    bmax[1] := Word(Trunc(qfac * maxy + 1)) or 1;
    bmax[2] := Word(Trunc(qfac * maxz + 1)) or 1;

    // Traverse tree
    base := m_nav.getPolyRefBase(tile);
    n := 0;
    while (node < &end) do
    begin
      overlap := dtOverlapQuantBounds(@bmin[0], @bmax[0], @node.bmin, @node.bmax[0]);
      isLeafNode := node.i >= 0;

      if (isLeafNode and overlap) then
      begin
        ref := base or TdtPolyRef(node.i);
        if (filter.passFilter(ref, tile, @tile.polys[node.i])) then
        begin
          if (n < maxPolys) then
          begin
            polys[n] := ref;
            Inc(n);
          end;
        end;
      end;

      if (overlap or isLeafNode) then
        Inc(node)
      else
      begin
        escapeIndex := -node.i;
        Inc(node, escapeIndex);
      end;
    end;

    Exit(n);
  end
  else
  begin
    n := 0;
    base := m_nav.getPolyRefBase(tile);
    for i := 0 to tile.header.polyCount - 1 do
    begin
      p := @tile.polys[i];
      // Do not return off-mesh connection polygons.
      if (p.getType() = DT_POLYTYPE_OFFMESH_CONNECTION) then
        continue;
      // Must pass filter
      ref := base or TdtPolyRef(i);
      if (not filter.passFilter(ref, tile, p)) then
        continue;
      // Calc polygon bounds.
      v := @tile.verts[p.verts[0]*3];
      dtVcopy(@bminf[0], v);
      dtVcopy(@bmaxf[0], v);
      for j := 1 to p.vertCount - 1 do
      begin
        v := @tile.verts[p.verts[j]*3];
        dtVmin(@bminf[0], v);
        dtVmax(@bmaxf[0], v);
      end;
      if (dtOverlapBounds(qmin,qmax, @bminf[0],@bmaxf[0])) then
      begin
        if (n < maxPolys) then
        begin
          polys[n] := ref;
          Inc(n);
        end;
      end;
    end;
    Exit(n);
  end;
end;

/// @par
///
/// If no polygons are found, the function will return #DT_SUCCESS with a
/// @p polyCount of zero.
///
/// If @p polys is too small to hold the entire result set, then the array will
/// be filled to capacity. The method of choosing which polygons from the
/// full set are included in the partial result set is undefined.
///
function TdtNavMeshQuery.queryPolygons(center, extents: PSingle; filter: TdtQueryFilter; polys: PdtPolyRef; polyCount: PInteger; maxPolys: Integer): TdtStatus;
const MAX_NEIS = 32;
var bmin,bmax: array [0..2] of Single; minx, miny, maxx, maxy: Integer; neis: array [0..MAX_NEIS-1] of PdtMeshTile; n,y,x,j,nneis: Integer;
begin
  //dtAssert(m_nav);

  dtVsub(@bmin[0], center, extents);
  dtVadd(@bmax[0], center, extents);

  // Find tiles the query touches.
  m_nav.calcTileLoc(@bmin[0], @minx, @miny);
  m_nav.calcTileLoc(@bmax[0], @maxx, @maxy);

  n := 0;
  for y := miny to maxy do
  begin
    for x := minx to maxx do
    begin
      nneis := m_nav.getTilesAt(x,y,@neis[0],MAX_NEIS);
      for j := 0 to nneis - 1 do
      begin
        Inc(n, queryPolygonsInTile(neis[j], @bmin[0], @bmax[0], filter, polys+n, maxPolys-n));
        if (n >= maxPolys) then
        begin
          polyCount^ := n;
          Exit(DT_SUCCESS or DT_BUFFER_TOO_SMALL);
        end;
      end;
    end;
  end;
  polyCount^ := n;

  Result := DT_SUCCESS;
end;

/// @par
///
/// If the end polygon cannot be reached through the navigation graph,
/// the last polygon in the path will be the nearest the end polygon.
///
/// If the path array is to small to hold the full result, it will be filled as
/// far as possible from the start polygon toward the end polygon.
///
/// The start and end positions are used to calculate traversal costs.
/// (The y-values impact the result.)
///
function TdtNavMeshQuery.findPath(startRef, endRef: TdtPolyRef;
            startPos, endPos: PSingle;
            filter: TdtQueryFilter;
            path: PdtPolyRef; pathCount: PInteger; maxPath: Integer): TdtStatus;
var startNode,lastBestNode,bestNode: PdtNode; lastBestNodeCost: Single; status: TdtStatus; bestRef,parentRef,neighbourRef: TdtPolyRef;
bestTile,parentTile,neighbourTile: PdtMeshTile; bestPoly,parentPoly,neighbourPoly: PdtPoly; i: Cardinal; crossSide: Byte;
neighbourNode,prev,node,next: PdtNode; cost,heuristic,curCost,endCost,total: Single; n: Integer;
begin
  Assert(m_nav <> nil);
  Assert(m_nodePool <> nil);
  Assert(m_openList <> nil);

  pathCount^ := 0;

  if (startRef = 0) or (endRef = 0) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  if (maxPath = 0) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // Validate input
  if (not m_nav.isValidPolyRef(startRef) or not m_nav.isValidPolyRef(endRef)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  if (startRef = endRef) then
  begin
    path[0] := startRef;
    pathCount^ := 1;
    Exit(DT_SUCCESS);
  end;

  m_nodePool.clear();
  m_openList.clear();

  startNode := m_nodePool.getNode(startRef);
  dtVcopy(@startNode.pos, startPos);
  startNode.pidx := 0;
  startNode.cost := 0;
  startNode.total := dtVdist(startPos, endPos) * H_SCALE;
  startNode.id := startRef;
  startNode.flags := DT_NODE_OPEN;
  m_openList.push(startNode);

  lastBestNode := startNode;
  lastBestNodeCost := startNode.total;

  status := DT_SUCCESS;

  while (not m_openList.empty()) do
  begin
    // Remove node from open list and put it in closed list.
    bestNode := m_openList.pop();
    bestNode.flags := bestNode.flags and not DT_NODE_OPEN;
    bestNode.flags := bestNode.flags or DT_NODE_CLOSED;

    // Reached the goal, stop searching.
    if (bestNode.id = endRef) then
    begin
      lastBestNode := bestNode;
      break;
    end;

    // Get current poly and tile.
    // The API input has been cheked already, skip checking internal data.
    bestRef := bestNode.id;
    bestTile := nil;
    bestPoly := nil;
    m_nav.getTileAndPolyByRefUnsafe(bestRef, @bestTile, @bestPoly);

    // Get parent poly and tile.
    parentRef := 0;
    parentTile := nil;
    parentPoly := nil;
    if (bestNode.pidx <> 0) then
      parentRef := m_nodePool.getNodeAtIdx(bestNode.pidx).id;
    if (parentRef <> 0) then
      m_nav.getTileAndPolyByRefUnsafe(parentRef, @parentTile, @parentPoly);

    i := bestPoly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      neighbourRef := bestTile.links[i].ref;

      // Skip invalid ids and do not expand back to where we came from.
      if (neighbourRef = 0) or (neighbourRef = parentRef) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Get neighbour poly and tile.
      // The API input has been cheked already, skip checking internal data.
      neighbourTile := nil;
      neighbourPoly := nil;
      m_nav.getTileAndPolyByRefUnsafe(neighbourRef, @neighbourTile, @neighbourPoly);

      if (not filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // deal explicitly with crossing tile boundaries
      crossSide := 0;
      if (bestTile.links[i].side <> $ff) then
        crossSide := bestTile.links[i].side shr 1;

      // get the node
      neighbourNode := m_nodePool.getNode(neighbourRef, crossSide);
      if (neighbourNode = nil) then
      begin
        status := status or DT_OUT_OF_NODES;
        begin
          i := bestTile.links[i].next;
          continue;
        end;
      end;

      // If the node is visited the first time, calculate node position.
      if (neighbourNode.flags = 0) then
      begin
        getEdgeMidPoint(bestRef, bestPoly, bestTile,
                neighbourRef, neighbourPoly, neighbourTile,
                @neighbourNode.pos);
      end;

      // Calculate cost and heuristic.
      cost := 0;
      heuristic := 0;

      // Special case for last node.
      if (neighbourRef = endRef) then
      begin
        // Cost
        curCost := filter.getCost(@bestNode.pos, @neighbourNode.pos,
                            parentRef, parentTile, parentPoly,
                            bestRef, bestTile, bestPoly,
                            neighbourRef, neighbourTile, neighbourPoly);
        endCost := filter.getCost(@neighbourNode.pos, endPos,
                            bestRef, bestTile, bestPoly,
                            neighbourRef, neighbourTile, neighbourPoly,
                            0, nil, nil);

        cost := bestNode.cost + curCost + endCost;
        heuristic := 0;
      end
      else
      begin
        // Cost
        curCost := filter.getCost(@bestNode.pos, @neighbourNode.pos,
                            parentRef, parentTile, parentPoly,
                            bestRef, bestTile, bestPoly,
                            neighbourRef, neighbourTile, neighbourPoly);
        cost := bestNode.cost + curCost;
        heuristic := dtVdist(@neighbourNode.pos, endPos)*H_SCALE;
      end;

      total := cost + heuristic;

      // The node is already in open list and the new result is worse, skip.
      if (neighbourNode.flags and DT_NODE_OPEN <> 0) and (total >= neighbourNode.total) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;
      // The node is already visited and process, and the new result is worse, skip.
      if (neighbourNode.flags and DT_NODE_CLOSED <> 0) and (total >= neighbourNode.total) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Add or update the node.
      neighbourNode.pidx := m_nodePool.getNodeIdx(bestNode);
      neighbourNode.id := neighbourRef;
      neighbourNode.flags := (neighbourNode.flags and not DT_NODE_CLOSED);
      neighbourNode.cost := cost;
      neighbourNode.total := total;

      if (neighbourNode.flags and DT_NODE_OPEN <> 0) then
      begin
        // Already in open, update node location.
        m_openList.modify(neighbourNode);
      end
      else
      begin
        // Put the node in open list.
        neighbourNode.flags := neighbourNode.flags or DT_NODE_OPEN;
        m_openList.push(neighbourNode);
      end;

      // Update nearest node to target so far.
      if (heuristic < lastBestNodeCost) then
      begin
        lastBestNodeCost := heuristic;
        lastBestNode := neighbourNode;
      end;

      i := bestTile.links[i].next;
    end;
  end;

  if (lastBestNode.id <> endRef) then
    status := status or DT_PARTIAL_RESULT;

  // Reverse the path.
  prev := nil;
  node := lastBestNode;
  repeat
    next := m_nodePool.getNodeAtIdx(node.pidx);
    node.pidx := m_nodePool.getNodeIdx(prev);
    prev := node;
    node := next;
  until (node = nil);

  // Store path
  node := prev;
  n := 0;
  repeat
    path[n] := node.id;
    Inc(n);
    if (n >= maxPath) then
    begin
      status := status or DT_BUFFER_TOO_SMALL;
      break;
    end;
    node := m_nodePool.getNodeAtIdx(node.pidx);
  until (node = nil);

  pathCount^ := n;

  Result := status;
end;

/// @par
///
/// @warning Calling any non-slice methods before calling finalizeSlicedFindPath()
/// or finalizeSlicedFindPathPartial() may result in corrupted data!
///
/// The @p filter pointer is stored and used for the duration of the sliced
/// path query.
///
function TdtNavMeshQuery.initSlicedFindPath(startRef, endRef: TdtPolyRef;
                startPos, endPos: PSingle;
                filter: TdtQueryFilter; options: Integer = 0): TdtStatus;
var tile: PdtMeshTile; agentRadius: Single; startNode: PdtNode;
begin
  Assert(m_nav <> nil);
  Assert(m_nodePool <> nil);
  Assert(m_openList <> nil);

  // Init path state.
  FillChar(m_query, sizeof(TdtQueryData), 0);
  m_query.status := DT_FAILURE;
  m_query.startRef := startRef;
  m_query.endRef := endRef;
  dtVcopy(@m_query.startPos[0], startPos);
  dtVcopy(@m_query.endPos[0], endPos);
  m_query.filter := filter;
  m_query.options := options;
  m_query.raycastLimitSqr := MaxSingle;

  if (startRef = 0) or (endRef = 0) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // Validate input
  if (not m_nav.isValidPolyRef(startRef) or not m_nav.isValidPolyRef(endRef)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // trade quality with performance?
  if (options and Byte(DT_FINDPATH_ANY_ANGLE)) <> 0 then
  begin
    // limiting to several times the character radius yields nice results. It is not sensitive
    // so it is enough to compute it from the first tile.
    tile := m_nav.getTileByRef(startRef);
    agentRadius := tile.header.walkableRadius;
    m_query.raycastLimitSqr := Sqr(agentRadius * DT_RAY_CAST_LIMIT_PROPORTIONS);
  end;

  if (startRef = endRef) then
  begin
    m_query.status := DT_SUCCESS;
    Exit(DT_SUCCESS);
  end;

  m_nodePool.clear();
  m_openList.clear();

  startNode := m_nodePool.getNode(startRef);
  dtVcopy(@startNode.pos, startPos);
  startNode.pidx := 0;
  startNode.cost := 0;
  startNode.total := dtVdist(startPos, endPos) * H_SCALE;
  startNode.id := startRef;
  startNode.flags := DT_NODE_OPEN;
  m_openList.push(startNode);

  m_query.status := DT_IN_PROGRESS;
  m_query.lastBestNode := startNode;
  m_query.lastBestNodeCost := startNode.total;

  Result := m_query.status;
end;

function TdtNavMeshQuery.updateSlicedFindPath(maxIter: Integer; doneIters: PInteger): TdtStatus;
var rayHit: TdtRaycastHit; iter: Integer; bestNode: PdtNode; details: TdtStatus;
bestRef,parentRef,grandpaRef,neighbourRef: TdtPolyRef; bestTile,parentTile,neighbourTile: PdtMeshTile; bestPoly,parentPoly,neighbourPoly: PdtPoly; parentNode,neighbourNode: PdtNode;
invalidParent,tryLOS: Boolean; i: Cardinal; cost,heuristic,curCost,endCost,total: Single;  foundShortCut: Boolean;
begin
  if (not dtStatusInProgress(m_query.status)) then
    Exit(m_query.status);

  // Make sure the request is still valid.
  if (not m_nav.isValidPolyRef(m_query.startRef) or not m_nav.isValidPolyRef(m_query.endRef)) then
  begin
    m_query.status := DT_FAILURE;
    Exit(DT_FAILURE);
  end;

  rayHit.maxPath := 0;

  iter := 0;
  while (iter < maxIter) and (not m_openList.empty()) do
  begin
    Inc(iter);

    // Remove node from open list and put it in closed list.
    bestNode := m_openList.pop();
    bestNode.flags := bestNode.flags and not DT_NODE_OPEN;
    bestNode.flags := bestNode.flags or DT_NODE_CLOSED;

    // Reached the goal, stop searching.
    if (bestNode.id = m_query.endRef) then
    begin
      m_query.lastBestNode := bestNode;
      details := m_query.status and DT_STATUS_DETAIL_MASK;
      m_query.status := DT_SUCCESS or details;
      if (doneIters <> nil) then
        doneIters^ := iter;
      Exit(m_query.status);
    end;

    // Get current poly and tile.
    // The API input has been cheked already, skip checking internal data.
    bestRef := bestNode.id;
    bestTile := nil;
    bestPoly := nil;
    if (dtStatusFailed(m_nav.getTileAndPolyByRef(bestRef, @bestTile, @bestPoly))) then
    begin
      // The polygon has disappeared during the sliced query, fail.
      m_query.status := DT_FAILURE;
      if (doneIters <> nil) then
        doneIters^ := iter;
      Exit(m_query.status);
    end;

    // Get parent and grand parent poly and tile.
    parentRef := 0; grandpaRef := 0;
    parentTile := nil;
    parentPoly := nil;
    parentNode := nil;
    if (bestNode.pidx <> 0) then
    begin
      parentNode := m_nodePool.getNodeAtIdx(bestNode.pidx);
      parentRef := parentNode.id;
      if (parentNode.pidx <> 0) then
        grandpaRef := m_nodePool.getNodeAtIdx(parentNode.pidx).id;
    end;
    if (parentRef <> 0) then
    begin
      invalidParent := dtStatusFailed(m_nav.getTileAndPolyByRef(parentRef, @parentTile, @parentPoly));
      if (invalidParent or ((grandpaRef <> 0) and not m_nav.isValidPolyRef(grandpaRef)) ) then
      begin
        // The polygon has disappeared during the sliced query, fail.
        m_query.status := DT_FAILURE;
        if (doneIters <> nil) then
          doneIters^ := iter;
        Exit(m_query.status);
      end;
    end;

    // decide whether to test raycast to previous nodes
    tryLOS := false;
    if (m_query.options and Byte(DT_FINDPATH_ANY_ANGLE)) <> 0 then
    begin
      if ((parentRef <> 0) and (dtVdistSqr(@parentNode.pos, @bestNode.pos) < m_query.raycastLimitSqr)) then
        tryLOS := true;
    end;

    i := bestPoly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      neighbourRef := bestTile.links[i].ref;

      // Skip invalid ids and do not expand back to where we came from.
      if (neighbourRef = 0) or (neighbourRef = parentRef) then
      begin
        i := bestTile.links[i].next;
        Continue;
      end;

      // Get neighbour poly and tile.
      // The API input has been cheked already, skip checking internal data.
      neighbourTile := nil;
      neighbourPoly := nil;
      m_nav.getTileAndPolyByRefUnsafe(neighbourRef, @neighbourTile, @neighbourPoly);

      if (not m_query.filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) then
      begin
        i := bestTile.links[i].next;
        Continue;
      end;

      // get the neighbor node
      neighbourNode := m_nodePool.getNode(neighbourRef, 0);
      if (neighbourNode = nil) then
      begin
        m_query.status := m_query.status or DT_OUT_OF_NODES;
        i := bestTile.links[i].next;
        Continue;
      end;

      // do not expand to nodes that were already visited from the same parent
      if (neighbourNode.pidx <> 0) and (neighbourNode.pidx = bestNode.pidx) then
      begin
        i := bestTile.links[i].next;
        Continue;
      end;

      // If the node is visited the first time, calculate node position.
      if (neighbourNode.flags = 0) then
      begin
        getEdgeMidPoint(bestRef, bestPoly, bestTile,
                neighbourRef, neighbourPoly, neighbourTile,
                @neighbourNode.pos);
      end;

      // Calculate cost and heuristic.
      cost := 0;
      heuristic := 0;

      // raycast parent
      foundShortCut := false;
      rayHit.pathCost := 0; rayHit.t := 0;
      if (tryLOS) then
      begin
        raycast(parentRef, @parentNode.pos, @neighbourNode.pos, m_query.filter, Byte(DT_RAYCAST_USE_COSTS), @rayHit, grandpaRef);
        foundShortCut := rayHit.t >= 1.0;
      end;

      // update move cost
      if (foundShortCut) then
      begin
        // shortcut found using raycast. Using shorter cost instead
        cost := parentNode.cost + rayHit.pathCost;
      end
      else
      begin
        // No shortcut found.
        curCost := m_query.filter.getCost(@bestNode.pos, @neighbourNode.pos,
                                parentRef, parentTile, parentPoly,
                              bestRef, bestTile, bestPoly,
                              neighbourRef, neighbourTile, neighbourPoly);
        cost := bestNode.cost + curCost;
      end;

      // Special case for last node.
      if (neighbourRef = m_query.endRef) then
      begin
        endCost := m_query.filter.getCost(@neighbourNode.pos, @m_query.endPos[0],
                                bestRef, bestTile, bestPoly,
                                neighbourRef, neighbourTile, neighbourPoly,
                                0, nil, nil);

        cost := cost + endCost;
        heuristic := 0;
      end
      else
      begin
        heuristic := dtVdist(@neighbourNode.pos, @m_query.endPos[0])*H_SCALE;
      end;

      total := cost + heuristic;

      // The node is already in open list and the new result is worse, skip.
      if (neighbourNode.flags and DT_NODE_OPEN <> 0) and (total >= neighbourNode.total) then
      begin
        i := bestTile.links[i].next;
        Continue;
      end;
      // The node is already visited and process, and the new result is worse, skip.
      if (neighbourNode.flags and DT_NODE_CLOSED <> 0) and (total >= neighbourNode.total) then
      begin
        i := bestTile.links[i].next;
        Continue;
      end;

      // Add or update the node.
      if foundShortCut then neighbourNode.pidx := bestNode.pidx else neighbourNode.pidx := m_nodePool.getNodeIdx(bestNode);
      neighbourNode.id := neighbourRef;
      neighbourNode.flags := (neighbourNode.flags and not (DT_NODE_CLOSED or DT_NODE_PARENT_DETACHED));
      neighbourNode.cost := cost;
      neighbourNode.total := total;
      if (foundShortCut) then
        neighbourNode.flags := (neighbourNode.flags or DT_NODE_PARENT_DETACHED);

      if (neighbourNode.flags and DT_NODE_OPEN <> 0) then
      begin
        // Already in open, update node location.
        m_openList.modify(neighbourNode);
      end
      else
      begin
        // Put the node in open list.
        neighbourNode.flags := neighbourNode.flags or DT_NODE_OPEN;
        m_openList.push(neighbourNode);
      end;

      // Update nearest node to target so far.
      if (heuristic < m_query.lastBestNodeCost) then
      begin
        m_query.lastBestNodeCost := heuristic;
        m_query.lastBestNode := neighbourNode;
      end;

      i := bestTile.links[i].next;
    end;
  end;

  // Exhausted all nodes, but could not find path.
  if (m_openList.empty()) then
  begin
    details := m_query.status and DT_STATUS_DETAIL_MASK;
    m_query.status := DT_SUCCESS or details;
  end;

  if (doneIters <> nil) then
    doneIters^ := iter;

  Result := m_query.status;
end;

function TdtNavMeshQuery.finalizeSlicedFindPath(path: PdtPolyRef; pathCount: PInteger; maxPath: Integer): TdtStatus;
var n,m: Integer; prev,node,next: PdtNode; prevRay,nextRay: Integer; status,details: TdtStatus; t: Single; normal: array [0..2] of Single;
begin
  pathCount^ := 0;

  if (dtStatusFailed(m_query.status)) then
  begin
    // Reset query.
    FillChar(m_query, sizeof(TdtQueryData), 0);
    Exit(DT_FAILURE);
  end;

  n := 0;

  if (m_query.startRef = m_query.endRef) then
  begin
    // Special case: the search starts and ends at same poly.
    path[n] := m_query.startRef;
    Inc(n);
  end
  else
  begin
    // Reverse the path.
    Assert(m_query.lastBestNode <> nil);

    if (m_query.lastBestNode.id <> m_query.endRef) then
      m_query.status := m_query.status or DT_PARTIAL_RESULT;

    prev := nil;
    node := m_query.lastBestNode;
    prevRay := 0;
    repeat
      next := m_nodePool.getNodeAtIdx(node.pidx);
      node.pidx := m_nodePool.getNodeIdx(prev);
      prev := node;
      nextRay := node.flags and DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
      node.flags := (node.flags and not DT_NODE_PARENT_DETACHED) or prevRay; // and store it in the reversed path's node
      prevRay := nextRay;
      node := next;
    until (node = nil);

    // Store path
    node := prev;
    repeat
      next := m_nodePool.getNodeAtIdx(node.pidx);
      status := 0;
      if (node.flags and DT_NODE_PARENT_DETACHED) <> 0 then
      begin
        status := raycast(node.id, @node.pos, @next.pos, m_query.filter, @t, @normal[0], path+n, @m, maxPath-n);
        Inc(n, m);
        // raycast ends on poly boundary and the path might include the next poly boundary.
        if (path[n-1] = next.id) then
          Dec(n); // remove to avoid duplicates
      end
      else
      begin
        path[n] := node.id;
        Inc(n);
        if (n >= maxPath) then
          status := DT_BUFFER_TOO_SMALL;
      end;

      if (status and DT_STATUS_DETAIL_MASK) <> 0 then
      begin
        m_query.status := m_query.status or (status and DT_STATUS_DETAIL_MASK);
        break;
      end;
      node := next;
    until (node = nil);
  end;

  details := m_query.status and DT_STATUS_DETAIL_MASK;

  // Reset query.
  FillChar(m_query, sizeof(TdtQueryData), 0);

  pathCount^ := n;

  Result := DT_SUCCESS or details;
end;

function TdtNavMeshQuery.finalizeSlicedFindPathPartial(existing: PdtPolyRef; existingSize: Integer;
                       path: PdtPolyRef; pathCount: PInteger; maxPath: Integer): TdtStatus;
var
  n, i, prevRay, nextRay, m: Integer; prev, node, next: PdtNode; status, details: TdtStatus; t: Single; normal: array [0..2] of Single;
begin
  pathCount^ := 0;

  if (existingSize = 0) then
  begin
    Exit(DT_FAILURE);
  end;

  if (dtStatusFailed(m_query.status)) then
  begin
    // Reset query.
    FillChar(m_query, sizeof(TdtQueryData), 0);
    Exit(DT_FAILURE);
  end;

  n := 0;

  if (m_query.startRef = m_query.endRef) then
  begin
    // Special case: the search starts and ends at same poly.
    path[n] := m_query.startRef;
    Inc(n);
  end
  else
  begin
    // Find furthest existing node that was visited.
    prev := nil;
    node := nil;
    for i := existingSize-1 downto 0 do
    begin
      m_nodePool.findNodes(existing[i], &node, 1);
      if (node <> nil) then
        break;
    end;

    if (node = nil) then
    begin
      m_query.status := m_query.status or DT_PARTIAL_RESULT;
      Assert(m_query.lastBestNode <> nil);
      node := m_query.lastBestNode;
    end;

    // Reverse the path.
    prevRay := 0;
    repeat
      next := m_nodePool.getNodeAtIdx(node.pidx);
      node.pidx := m_nodePool.getNodeIdx(prev);
      prev := node;
      nextRay := node.flags and DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
      node.flags := (node.flags and not DT_NODE_PARENT_DETACHED) or prevRay; // and store it in the reversed path's node
      prevRay := nextRay;
      node := next;
    until (node = nil);

    // Store path
    node := prev;
    repeat
      next := m_nodePool.getNodeAtIdx(node.pidx);
      status := 0;
      if (node.flags and DT_NODE_PARENT_DETACHED) <> 0 then
      begin
        status := raycast(node.id, @node.pos[0], @next.pos[0], m_query.filter, @t, @normal[0], path+n, @m, maxPath-n);
        Inc(n, m);
        // raycast ends on poly boundary and the path might include the next poly boundary.
        if (path[n-1] = next.id) then
          Dec(n); // remove to avoid duplicates
      end
      else
      begin
        path[n] := node.id;
        Inc(n);
        if (n >= maxPath) then
          status := DT_BUFFER_TOO_SMALL;
      end;

      if (status and DT_STATUS_DETAIL_MASK) <> 0 then
      begin
        m_query.status := m_query.status or (status and DT_STATUS_DETAIL_MASK);
        break;
      end;
      node := next;
    until (node = nil);
  end;

  details := m_query.status and DT_STATUS_DETAIL_MASK;

  // Reset query.
  FillChar(m_query, sizeof(TdtQueryData), 0);

  pathCount^ := n;

  Result := DT_SUCCESS or details;
end;


function TdtNavMeshQuery.appendVertex(pos: PSingle; flags: Integer; ref: TdtPolyRef;
              straightPath: PSingle; straightPathFlags: PByte; straightPathRefs: PdtPolyRef;
              straightPathCount: PInteger; maxStraightPath: Integer): TdtStatus;
begin
  if (straightPathCount^ > 0) and dtVequal(@straightPath[((straightPathCount^)-1)*3], pos) then
  begin
    // The vertices are equal, update flags and poly.
    if (straightPathFlags <> nil) then
      straightPathFlags[(straightPathCount^)-1] := flags;
    if (straightPathRefs <> nil) then
      straightPathRefs[(straightPathCount^)-1] := ref;
  end
  else
  begin
    // Append new vertex.
    dtVcopy(@straightPath[(straightPathCount^)*3], pos);
    if (straightPathFlags <> nil) then
      straightPathFlags[(straightPathCount^)] := flags;
    if (straightPathRefs <> nil) then
      straightPathRefs[(straightPathCount^)] := ref;
    Inc(straightPathCount^);
    // If reached end of path or there is no space to append more vertices, return.
    if (flags = Byte(DT_STRAIGHTPATH_END)) or (( straightPathCount^) >= maxStraightPath) then
    begin
      Exit(DT_SUCCESS or IfThen(((straightPathCount^) >= maxStraightPath), DT_BUFFER_TOO_SMALL, 0));
    end;
  end;
  Result := DT_IN_PROGRESS;
end;

function TdtNavMeshQuery.appendPortals(startIdx, endIdx: Integer; endPos: PSingle; path: PdtPolyRef;
               straightPath: PSingle; straightPathFlags: PByte; straightPathRefs: PdtPolyRef;
               straightPathCount: PInteger; maxStraightPath, options: Integer): TdtStatus;
var startPos: PSingle; stat: TdtStatus; i: Integer; from,&to: TdtPolyRef; fromTile,toTile: PdtMeshTile; fromPoly,toPoly: PdtPoly;
left,right,pt: array [0..2] of Single; s,t: Single;
begin
  startPos := @straightPath[(straightPathCount^-1)*3];
  // Append or update last vertex
  stat := 0;
  for i := startIdx to endIdx - 1 do
  begin
    // Calculate portal
    from := path[i];
    fromTile := nil;
    fromPoly := nil;
    if (dtStatusFailed(m_nav.getTileAndPolyByRef(from, @fromTile, @fromPoly))) then
      Exit(DT_FAILURE or DT_INVALID_PARAM);

    &to := path[i+1];
    toTile := nil;
    toPoly := nil;
    if (dtStatusFailed(m_nav.getTileAndPolyByRef(&to, @toTile, @toPoly))) then
      Exit(DT_FAILURE or DT_INVALID_PARAM);

    if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, &to, toPoly, toTile, @left[0], @right[0]))) then
      break;

    if (options and Byte(DT_STRAIGHTPATH_AREA_CROSSINGS)) <> 0 then
    begin
      // Skip intersection if only area crossings are requested.
      if (fromPoly.getArea() = toPoly.getArea()) then
        continue;
    end;

    // Append intersection
    if (dtIntersectSegSeg2D(startPos, endPos, @left[0], @right[0], @s, @t)) then
    begin
      dtVlerp(@pt[0], @left[0],@right[0], t);

      stat := appendVertex(@pt[0], 0, path[i+1],
                straightPath, straightPathFlags, straightPathRefs,
                straightPathCount, maxStraightPath);
      if (stat <> DT_IN_PROGRESS) then
        Exit(stat);
    end;
  end;
  Result := DT_IN_PROGRESS;
end;

/// @par
///
/// This method peforms what is often called 'string pulling'.
///
/// The start position is clamped to the first polygon in the path, and the
/// end position is clamped to the last. So the start and end positions should
/// normally be within or very near the first and last polygons respectively.
///
/// The returned polygon references represent the reference id of the polygon
/// that is entered at the associated path position. The reference id associated
/// with the end point will always be zero.  This allows, for example, matching
/// off-mesh link points to their representative polygons.
///
/// If the provided result buffers are too small for the entire result set,
/// they will be filled as far as possible from the start toward the end
/// position.
///
function TdtNavMeshQuery.findStraightPath(startPos, endPos: PSingle;
                      path: PdtPolyRef; pathSize: Integer;
                      straightPath: PSingle; straightPathFlags: PByte; straightPathRefs: PdtPolyRef;
                      straightPathCount: PInteger; maxStraightPath: Integer; options: Integer = 0): TdtStatus;
var stat: TdtStatus; closestStartPos,closestEndPos,portalApex,portalLeft,portalRight: array [0..2] of Single;
apexIndex,leftIndex,rightIndex: Integer; leftPolyType,rightPolyType: Byte; leftPolyRef,rightPolyRef: TdtPolyRef; i: Integer;
left,right: array [0..2] of Single; fromType,toType: Byte; t: Single; flags: Byte; ref: TdtPolyRef;
begin
  //dtAssert(m_nav);

  straightPathCount^ := 0;

  if (maxStraightPath = 0) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  if (path[0] = 0) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  stat := 0;

  // TODO: Should this be callers responsibility?
  if (dtStatusFailed(closestPointOnPolyBoundary(path[0], startPos, @closestStartPos[0]))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  if (dtStatusFailed(closestPointOnPolyBoundary(path[pathSize-1], endPos, @closestEndPos[0]))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // Add start point.
  stat := appendVertex(@closestStartPos[0], Byte(DT_STRAIGHTPATH_START), path[0],
            straightPath, straightPathFlags, straightPathRefs,
            straightPathCount, maxStraightPath);
  if (stat <> DT_IN_PROGRESS) then
    Exit(stat);

  if (pathSize > 1) then
  begin
    dtVcopy(@portalApex[0], @closestStartPos[0]);
    dtVcopy(@portalLeft[0], @portalApex[0]);
    dtVcopy(@portalRight[0], @portalApex[0]);
    apexIndex := 0;
    leftIndex := 0;
    rightIndex := 0;

    leftPolyType := 0;
    rightPolyType := 0;

    leftPolyRef := path[0];
    rightPolyRef := path[0];

    i := 0;
    while (i < pathSize) do
    begin
      if (i+1 < pathSize) then
      begin
        // Next portal.
        if (dtStatusFailed(getPortalPoints(path[i], path[i+1], @left[0], @right[0], @fromType, @toType))) then
        begin
          // Failed to get portal points, in practice this means that path[i+1] is invalid polygon.
          // Clamp the end point to path[i], and return the path so far.

          if (dtStatusFailed(closestPointOnPolyBoundary(path[i], endPos, @closestEndPos[0]))) then
          begin
            // This should only happen when the first polygon is invalid.
            Exit(DT_FAILURE or DT_INVALID_PARAM);
          end;

          // Apeend portals along the current straight path segment.
          if (options and (Byte(DT_STRAIGHTPATH_AREA_CROSSINGS) or Byte(DT_STRAIGHTPATH_ALL_CROSSINGS)) <> 0) then
          begin
            stat := appendPortals(apexIndex, i, @closestEndPos[0], path,
                       straightPath, straightPathFlags, straightPathRefs,
                       straightPathCount, maxStraightPath, options);
          end;

          stat := appendVertex(@closestEndPos[0], 0, path[i],
                    straightPath, straightPathFlags, straightPathRefs,
                    straightPathCount, maxStraightPath);

          Exit(DT_SUCCESS or DT_PARTIAL_RESULT or IfThen((straightPathCount^ >= maxStraightPath), DT_BUFFER_TOO_SMALL, 0));
        end;

        // If starting really close the portal, advance.
        if (i = 0) then
        begin
          if (dtDistancePtSegSqr2D(@portalApex[0], @left[0], @right[0], @t) < Sqr(0.001)) then
          begin
            Inc(i);
            continue;
          end;
        end;
      end
      else
      begin
        // End of the path.
        dtVcopy(@left[0], @closestEndPos[0]);
        dtVcopy(@right[0], @closestEndPos[0]);

        fromType := DT_POLYTYPE_GROUND; toType := DT_POLYTYPE_GROUND;
      end;

      // Right vertex.
      if (dtTriArea2D(@portalApex[0], @portalRight[0], @right[0]) <= 0.0) then
      begin
        if (dtVequal(@portalApex[0], @portalRight[0]) or (dtTriArea2D(@portalApex[0], @portalLeft[0], @right[0]) > 0.0)) then
        begin
          dtVcopy(@portalRight[0], @right[0]);
          if (i+1 < pathSize) then rightPolyRef := path[i+1] else rightPolyRef := 0;
          rightPolyType := toType;
          rightIndex := i;
        end
        else
        begin
          // Append portals along the current straight path segment.
          if (options and (Byte(DT_STRAIGHTPATH_AREA_CROSSINGS) or Byte(DT_STRAIGHTPATH_ALL_CROSSINGS)) <> 0) then
          begin
            stat := appendPortals(apexIndex, leftIndex, @portalLeft[0], path,
                       straightPath, straightPathFlags, straightPathRefs,
                       straightPathCount, maxStraightPath, options);
            if (stat <> DT_IN_PROGRESS) then
              Exit(stat);
          end;

          dtVcopy(@portalApex[0], @portalLeft[0]);
          apexIndex := leftIndex;

          flags := 0;
          if (leftPolyRef = 0) then
            flags := Byte(DT_STRAIGHTPATH_END)
          else if (leftPolyType = DT_POLYTYPE_OFFMESH_CONNECTION) then
            flags := Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION);
          ref := leftPolyRef;

          // Append or update vertex
          stat := appendVertex(@portalApex[0], flags, ref,
                    straightPath, straightPathFlags, straightPathRefs,
                    straightPathCount, maxStraightPath);
          if (stat <> DT_IN_PROGRESS) then
            Exit(stat);

          dtVcopy(@portalLeft[0], @portalApex[0]);
          dtVcopy(@portalRight[0], @portalApex[0]);
          leftIndex := apexIndex;
          rightIndex := apexIndex;

          // Restart
          i := apexIndex;

          Inc(i);
           continue;
        end;
      end;

      // Left vertex.
      if (dtTriArea2D(@portalApex[0], @portalLeft[0], @left[0]) >= 0.0) then
      begin
        if (dtVequal(@portalApex[0], @portalLeft[0])) or (dtTriArea2D(@portalApex[0], @portalRight[0], @left[0]) < 0.0) then
        begin
          dtVcopy(@portalLeft[0], @left[0]);
          if (i+1 < pathSize) then leftPolyRef := path[i+1] else leftPolyRef := 0;
          leftPolyType := toType;
          leftIndex := i;
        end
        else
        begin
          // Append portals along the current straight path segment.
          if (options and (Byte(DT_STRAIGHTPATH_AREA_CROSSINGS) or Byte(DT_STRAIGHTPATH_ALL_CROSSINGS)) <> 0) then
          begin
            stat := appendPortals(apexIndex, rightIndex, @portalRight[0], path,
                       straightPath, straightPathFlags, straightPathRefs,
                       straightPathCount, maxStraightPath, options);
            if (stat <> DT_IN_PROGRESS) then
              Exit(stat);
          end;

          dtVcopy(@portalApex[0], @portalRight[0]);
          apexIndex := rightIndex;

          flags := 0;
          if (rightPolyRef = 0) then
            flags := Byte(DT_STRAIGHTPATH_END)
          else if (rightPolyType = DT_POLYTYPE_OFFMESH_CONNECTION) then
            flags := Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION);
          ref := rightPolyRef;

          // Append or update vertex
          stat := appendVertex(@portalApex[0], flags, ref,
                    straightPath, straightPathFlags, straightPathRefs,
                    straightPathCount, maxStraightPath);
          if (stat <> DT_IN_PROGRESS) then
            Exit(stat);

          dtVcopy(@portalLeft[0], @portalApex[0]);
          dtVcopy(@portalRight[0], @portalApex[0]);
          leftIndex := apexIndex;
          rightIndex := apexIndex;

          // Restart
          i := apexIndex;

          Inc(i);
           continue;
        end;
      end;

      Inc(i);
    end;

    // Append portals along the current straight path segment.
    if (options and (Byte(DT_STRAIGHTPATH_AREA_CROSSINGS) or Byte(DT_STRAIGHTPATH_ALL_CROSSINGS)) <> 0) then
    begin
      stat := appendPortals(apexIndex, pathSize-1, @closestEndPos[0], path,
                 straightPath, straightPathFlags, straightPathRefs,
                 straightPathCount, maxStraightPath, options);
      if (stat <> DT_IN_PROGRESS) then
        Exit(stat);
    end;
  end;

  stat := appendVertex(@closestEndPos[0], Byte(DT_STRAIGHTPATH_END), 0,
            straightPath, straightPathFlags, straightPathRefs,
            straightPathCount, maxStraightPath);

  Result := DT_SUCCESS or IfThen((straightPathCount^ >= maxStraightPath), DT_BUFFER_TOO_SMALL, 0);
end;

/// @par
///
/// This method is optimized for small delta movement and a small number of
/// polygons. If used for too great a distance, the result set will form an
/// incomplete path.
///
/// @p resultPos will equal the @p endPos if the end is reached.
/// Otherwise the closest reachable position will be returned.
///
/// @p resultPos is not projected onto the surface of the navigation
/// mesh. Use #getPolyHeight if this is needed.
///
/// This method treats the end position in the same manner as
/// the #raycast method. (As a 2D point.) See that method's documentation
/// for details.
///
/// If the @p visited array is too small to hold the entire result set, it will
/// be filled as far as possible from the start position toward the end
/// position.
///
function TdtNavMeshQuery.moveAlongSurface(startRef: TdtPolyRef; startPos, endPos: PSingle;
                filter: TdtQueryFilter;
                resultPos: PSingle; visited: PdtPolyRef; visitedCount: PInteger; maxVisitedSize: Integer): TdtStatus;
const MAX_STACK = 48;
const MAX_NEIS = 8;
var status: TdtStatus; stack: array [0..MAX_STACK-1] of PdtNode; nstack,i,j,nverts: Integer; startNode,bestNode,curNode: PdtNode;
bestPos,searchPos: array [0..2] of Single; bestDist,searchRadSqr: Single; verts: array [0..DT_VERTS_PER_POLYGON*3-1] of Single;
curRef: TdtPolyRef; curTile,neiTile: PdtMeshTile; curPoly,neiPoly: PdtPoly; nneis: Integer; neis: array [0..MAX_NEIS-1] of TdtPolyRef; k: Cardinal;
link: PdtLink; idx: Cardinal; ref: TdtPolyRef; vj,vi: PSingle; tseg,distSqr: Single; neighbourNode: PdtNode; n: Integer;
prev,node,next: PdtNode;
begin
  //dtAssert(m_nav);
  //dtAssert(m_tinyNodePool);

  visitedCount^ := 0;

  // Validate input
  if (startRef = 0) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (not m_nav.isValidPolyRef(startRef)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  status := DT_SUCCESS;

  nstack := 0;

  m_tinyNodePool.clear();

  startNode := m_tinyNodePool.getNode(startRef);
  startNode.pidx := 0;
  startNode.cost := 0;
  startNode.total := 0;
  startNode.id := startRef;
  startNode.flags := DT_NODE_CLOSED;
  stack[nstack] := startNode;
  Inc(nstack);

  bestDist := MaxSingle;
  bestNode := nil;
  dtVcopy(@bestPos[0], startPos);

  // Search constraints  F
  dtVlerp(@searchPos[0], startPos, endPos, 0.5);
  searchRadSqr := Sqr(dtVdist(startPos, endPos)/2.0 + 0.001);

  while (nstack <> 0) do
  begin
    // Pop front.
    curNode := stack[0];
    for i := 0 to nstack-1 - 1 do
      stack[i] := stack[i+1];
    Dec(nstack);

    // Get poly and tile.
    // The API input has been cheked already, skip checking internal data.
    curRef := curNode.id;
    curTile := nil;
    curPoly := nil;
    m_nav.getTileAndPolyByRefUnsafe(curRef, @curTile, @curPoly);

    // Collect vertices.
    nverts := curPoly.vertCount;
    for i := 0 to nverts - 1 do
      dtVcopy(@verts[i*3], @curTile.verts[curPoly.verts[i]*3]);

    // If target is inside the poly, stop search.
    if (dtPointInPolygon(endPos, @verts[0], nverts)) then
    begin
      bestNode := curNode;
      dtVcopy(@bestPos[0], endPos);
      break;
    end;

    // Find wall edges and find nearest point inside the walls.
    i := 0; j := curPoly.vertCount-1;
    while (i < curPoly.vertCount) do
    begin
      // Find links to neighbours.
      nneis := 0;

      if (curPoly.neis[j] and DT_EXT_LINK) <> 0 then
      begin
        // Tile border.
        k := curPoly.firstLink;
        while (k <> DT_NULL_LINK) do
        begin
          link := @curTile.links[k];
          if (link.edge = j) then
          begin
            if (link.ref <> 0) then
            begin
              neiTile := nil;
              neiPoly := nil;
              m_nav.getTileAndPolyByRefUnsafe(link.ref, @neiTile, @neiPoly);
              if (filter.passFilter(link.ref, neiTile, neiPoly)) then
              begin
                if (nneis < MAX_NEIS) then
                begin
                  neis[nneis] := link.ref;
                  Inc(nneis);
                end;
              end;
            end;
          end;
          k := curTile.links[k].next;
        end;
      end
      else if (curPoly.neis[j] <> 0) then
      begin
        idx := (curPoly.neis[j]-1);
        ref := m_nav.getPolyRefBase(curTile) or idx;
        if (filter.passFilter(ref, curTile, @curTile.polys[idx])) then
        begin
          // Internal edge, encode id.
          neis[nneis] := ref;
          Inc(nneis);
        end;
      end;

      if (nneis = 0) then
      begin
        // Wall edge, calc distance.
        vj := @verts[j*3];
        vi := @verts[i*3];
        distSqr := dtDistancePtSegSqr2D(endPos, vj, vi, @tseg);
        if (distSqr < bestDist) then
        begin
                    // Update nearest distance.
          dtVlerp(@bestPos[0], vj,vi, tseg);
          bestDist := distSqr;
          bestNode := curNode;
        end;
      end
      else
      begin
        for k := 0 to nneis - 1 do
        begin
          // Skip if no node can be allocated.
          neighbourNode := m_tinyNodePool.getNode(neis[k]);
          if (neighbourNode = nil) then
            continue;
          // Skip if already visited.
          if (neighbourNode.flags and DT_NODE_CLOSED) <> 0 then
            continue;

          // Skip the link if it is too far from search constraint.
          // TODO: Maybe should use getPortalPoints(), but this one is way faster.
          vj := @verts[j*3];
          vi := @verts[i*3];
          distSqr := dtDistancePtSegSqr2D(@searchPos[0], vj, vi, @tseg);
          if (distSqr > searchRadSqr) then
            continue;

          // Mark as the node as visited and push to queue.
          if (nstack < MAX_STACK) then
          begin
            neighbourNode.pidx := m_tinyNodePool.getNodeIdx(curNode);
            neighbourNode.flags := neighbourNode.flags or DT_NODE_CLOSED;
            stack[nstack] := neighbourNode;
            Inc(nstack);
          end;
        end;
      end;

      j := i;
      Inc(i);
    end;
  end;

  n := 0;
  if (bestNode <> nil) then
  begin
    // Reverse the path.
    prev := nil;
    node := bestNode;
    repeat
      next := m_tinyNodePool.getNodeAtIdx(node.pidx);
      node.pidx := m_tinyNodePool.getNodeIdx(prev);
      prev := node;
      node := next;
    until (node = nil);

    // Store result
    node := prev;
    repeat
      visited[n] := node.id;
      Inc(n);
      if (n >= maxVisitedSize) then
      begin
        status := status or DT_BUFFER_TOO_SMALL;
        break;
      end;
      node := m_tinyNodePool.getNodeAtIdx(node.pidx);
    until (node = nil);
  end;

  dtVcopy(resultPos, @bestPos[0]);

  visitedCount^ := n;

  Result := status;
end;


function TdtNavMeshQuery.getPortalPoints(from, &to: TdtPolyRef; left, right: PSingle;
               fromType, toType: PByte): TdtStatus;
var fromTile,toTile: PdtMeshTile; fromPoly,toPoly: PdtPoly;
begin
  //dtAssert(m_nav);

  fromTile := nil;
  fromPoly := nil;
  if (dtStatusFailed(m_nav.getTileAndPolyByRef(from, @fromTile, @fromPoly))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  fromType^ := fromPoly.getType;

  toTile := nil;
  toPoly := nil;
  if (dtStatusFailed(m_nav.getTileAndPolyByRef(&to, @toTile, @toPoly))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  toType^ := toPoly.getType;

  Result := getPortalPoints(from, fromPoly, fromTile, &to, toPoly, toTile, left, right);
end;

// Returns portal points between two polygons.
function TdtNavMeshQuery.getPortalPoints(from: TdtPolyRef; fromPoly: PdtPoly; fromTile: PdtMeshTile;
               &to: TdtPolyRef; toPoly: PdtPoly; toTile: PdtMeshTile;
               left, right: PSingle): TdtStatus;
var link: PdtLink; i: Cardinal; v,v0,v1: Integer; s,tmin,tmax: Single;
begin
  // Find the link that points to the 'to' polygon.
  link := nil;
  i := fromPoly.firstLink;
  while (i <> DT_NULL_LINK) do
  begin
    if (fromTile.links[i].ref = &to) then
    begin
      link := @fromTile.links[i];
      break;
    end;

    i := fromTile.links[i].next;
  end;
  if (link = nil) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // Handle off-mesh connections.
  if (fromPoly.getType = DT_POLYTYPE_OFFMESH_CONNECTION) then
  begin
    // Find link that points to first vertex.
    i := fromPoly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      if (fromTile.links[i].ref = &to) then
      begin
        v := fromTile.links[i].edge;
        dtVcopy(left, @fromTile.verts[fromPoly.verts[v]*3]);
        dtVcopy(right, @fromTile.verts[fromPoly.verts[v]*3]);
        Exit(DT_SUCCESS);
      end;

      i := fromTile.links[i].next;
    end;
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  end;

  if (toPoly.getType = DT_POLYTYPE_OFFMESH_CONNECTION) then
  begin
    i := toPoly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      if (toTile.links[i].ref = from) then
      begin
        v := toTile.links[i].edge;
        dtVcopy(left, @toTile.verts[toPoly.verts[v]*3]);
        dtVcopy(right, @toTile.verts[toPoly.verts[v]*3]);
        Exit(DT_SUCCESS);
      end;

      i := toTile.links[i].next;
    end;
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  end;

  // Find portal vertices.
  v0 := fromPoly.verts[link.edge];
  v1 := fromPoly.verts[(link.edge+1) mod fromPoly.vertCount];
  dtVcopy(left, @fromTile.verts[v0*3]);
  dtVcopy(right, @fromTile.verts[v1*3]);

  // If the link is at tile boundary, dtClamp the vertices to
  // the link width.
  if (link.side <> $ff) then
  begin
    // Unpack portal limits.
    if (link.bmin <> 0) or (link.bmax <> 255) then
    begin
      s := 1.0/255.0;
      tmin := link.bmin*s;
      tmax := link.bmax*s;
      dtVlerp(left, @fromTile.verts[v0*3], @fromTile.verts[v1*3], tmin);
      dtVlerp(right, @fromTile.verts[v0*3], @fromTile.verts[v1*3], tmax);
    end;
  end;

  Result := DT_SUCCESS;
end;

// Returns edge mid point between two polygons.
function TdtNavMeshQuery.getEdgeMidPoint(from, &to: TdtPolyRef; mid: PSingle): TdtStatus;
var left, right: array [0..2] of Single; fromType, toType: Byte;
begin
  if (dtStatusFailed(getPortalPoints(from, &to, @left[0],@right[0], @fromType, @toType))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  mid[0] := (left[0]+right[0])*0.5;
  mid[1] := (left[1]+right[1])*0.5;
  mid[2] := (left[2]+right[2])*0.5;
  Result := DT_SUCCESS;
end;

function TdtNavMeshQuery.getEdgeMidPoint(from: TdtPolyRef; fromPoly: PdtPoly; fromTile: PdtMeshTile;
               &to: TdtPolyRef; toPoly: PdtPoly; toTile: PdtMeshTile;
               mid: PSingle): TdtStatus;
var left, right: array [0..2] of Single;
begin
  if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, &to, toPoly, toTile, @left[0], @right[0]))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  mid[0] := (left[0]+right[0])*0.5;
  mid[1] := (left[1]+right[1])*0.5;
  mid[2] := (left[2]+right[2])*0.5;
  Result := DT_SUCCESS;
end;



/// @par
///
/// This method is meant to be used for quick, short distance checks.
///
/// If the path array is too small to hold the result, it will be filled as
/// far as possible from the start postion toward the end position.
///
/// <b>Using the Hit Parameter (t)</b>
///
/// If the hit parameter is a very high value (FLT_MAX), then the ray has hit
/// the end position. In this case the path represents a valid corridor to the
/// end position and the value of @p hitNormal is undefined.
///
/// If the hit parameter is zero, then the start position is on the wall that
/// was hit and the value of @p hitNormal is undefined.
///
/// If 0 < t < 1.0 then the following applies:
///
/// @code
/// distanceToHitBorder := distanceToEndPosition * t
/// hitPoint := startPos + (endPos - startPos) * t
/// @endcode
///
/// <b>Use Case Restriction</b>
///
/// The raycast ignores the y-value of the end position. (2D check.) This
/// places significant limits on how it can be used. For example:
///
/// Consider a scene where there is a main floor with a second floor balcony
/// that hangs over the main floor. So the first floor mesh extends below the
/// balcony mesh. The start position is somewhere on the first floor. The end
/// position is on the balcony.
///
/// The raycast will search toward the end position along the first floor mesh.
/// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
/// (no wall hit), meaning it reached the end position. This is one example of why
/// this method is meant for short distance checks.
///
function TdtNavMeshQuery.raycast(startRef: TdtPolyRef; startPos, endPos: PSingle;
           filter: TdtQueryFilter;
           t, hitNormal: PSingle; path: PdtPolyRef; pathCount: PInteger; maxPath: Integer): TdtStatus;
var hit: TdtRaycastHit; status: TdtStatus;
begin
  hit.path := path;
  hit.maxPath := maxPath;

  status := raycast(startRef, startPos, endPos, filter, 0, @hit);

  t^ := hit.t;
  if (hitNormal <> nil) then
    dtVcopy(hitNormal, @hit.hitNormal[0]);
  if (pathCount <> nil) then
    pathCount^ := hit.pathCount;

  Result := status;
end;


/// @par
///
/// This method is meant to be used for quick, short distance checks.
///
/// If the path array is too small to hold the result, it will be filled as
/// far as possible from the start postion toward the end position.
///
/// <b>Using the Hit Parameter t of RaycastHit</b>
///
/// If the hit parameter is a very high value (FLT_MAX), then the ray has hit
/// the end position. In this case the path represents a valid corridor to the
/// end position and the value of @p hitNormal is undefined.
///
/// If the hit parameter is zero, then the start position is on the wall that
/// was hit and the value of @p hitNormal is undefined.
///
/// If 0 < t < 1.0 then the following applies:
///
/// @code
/// distanceToHitBorder := distanceToEndPosition * t
/// hitPoint := startPos + (endPos - startPos) * t
/// @endcode
///
/// <b>Use Case Restriction</b>
///
/// The raycast ignores the y-value of the end position. (2D check.) This
/// places significant limits on how it can be used. For example:
///
/// Consider a scene where there is a main floor with a second floor balcony
/// that hangs over the main floor. So the first floor mesh extends below the
/// balcony mesh. The start position is somewhere on the first floor. The end
/// position is on the balcony.
///
/// The raycast will search toward the end position along the first floor mesh.
/// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
/// (no wall hit), meaning it reached the end position. This is one example of why
/// this method is meant for short distance checks.
///
function TdtNavMeshQuery.raycast(startRef: TdtPolyRef; startPos, endPos: PSingle;
           filter: TdtQueryFilter; options: Cardinal;
           hit: PdtRaycastHit; prevRef: TdtPolyRef = 0): TdtStatus;
var dir,curPos,lastPos,eDir,diff: array [0..2] of Single; verts: array [0..DT_VERTS_PER_POLYGON*3+3-1] of Single; n: Integer; status: TdtStatus;
prevTile,tile,nextTile: PdtMeshTile; prevPoly,poly,nextPoly: PdtPoly; curRef,nextRef: TdtPolyRef; nv: Integer; i: Cardinal;
tmin, tmax: Single; segMin, segMax: Integer; link: PdtLink; v0,v1: Integer; left,right: PSingle; s,lmin,lmax,z,x: Single;
e1,e2: PSingle; a,b: Integer; va,vb: PSingle; dx,dz: Single;
begin
  //dtAssert(m_nav);

  hit.t := 0;
  hit.pathCount := 0;
  hit.pathCost := 0;

  // Validate input
  if (startRef = 0) or not m_nav.isValidPolyRef(startRef) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (prevRef <> 0) and not m_nav.isValidPolyRef(prevRef) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  n := 0;

  dtVcopy(@curPos[0], startPos);
  dtVsub(@dir[0], endPos, startPos);
  dtVset(@hit.hitNormal[0], 0, 0, 0);

  status := DT_SUCCESS;

  // The API input has been checked already, skip checking internal data.
  nextRef := startRef; curRef := startRef;
  tile := nil;
  poly := nil;
  m_nav.getTileAndPolyByRefUnsafe(curRef, @tile, @poly);
  nextTile := tile; prevTile := tile;
  nextPoly := poly; prevPoly := poly;
  if (prevRef <> 0) then
    m_nav.getTileAndPolyByRefUnsafe(prevRef, @prevTile, @prevPoly);

  while (curRef <> 0) do
  begin
    // Cast ray against current polygon.

    // Collect vertices.
    nv := 0;
    for i := 0 to Integer(poly.vertCount) - 1 do
    begin
      dtVcopy(@verts[nv*3], @tile.verts[poly.verts[i]*3]);
      Inc(nv);
    end;

    if (not dtIntersectSegmentPoly2D(startPos, endPos, @verts[0], nv, @tmin, @tmax, @segMin, @segMax)) then
    begin
      // Could not hit the polygon, keep the old t and report hit.
      hit.pathCount := n;
      Exit(status);
    end;
    // Keep track of furthest t so far.
    if (tmax > hit.t) then
      hit.t := tmax;

    // Store visited polygons.
    if (n < hit.maxPath) then
    begin
      hit.path[n] := curRef;
      Inc(n);
    end
    else
      status := status or DT_BUFFER_TOO_SMALL;

    // Ray end is completely inside the polygon.
    if (segMax = -1) then
    begin
      hit.t := MaxSingle;
      hit.pathCount := n;

      // add the cost
      if (options and Byte(DT_RAYCAST_USE_COSTS) <> 0) then
        hit.pathCost := hit.pathCost + filter.getCost(@curPos[0], endPos, prevRef, prevTile, prevPoly, curRef, tile, poly, curRef, tile, poly);
      Exit(status);
    end;

    // Follow neighbours.
    nextRef := 0;

    i := poly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      link := @tile.links[i];

      // Find link which contains this edge.
      if (link.edge <> segMax) then
      begin
        i := tile.links[i].next;
        continue;
      end;

      // Get pointer to the next polygon.
      nextTile := nil;
      nextPoly := nil;
      m_nav.getTileAndPolyByRefUnsafe(link.ref, @nextTile, @nextPoly);

      // Skip off-mesh connections.
      if (nextPoly.getType() = DT_POLYTYPE_OFFMESH_CONNECTION) then
      begin
        i := tile.links[i].next;
        continue;
      end;

      // Skip links based on filter.
      if (not filter.passFilter(link.ref, nextTile, nextPoly)) then
      begin
        i := tile.links[i].next;
        continue;
      end;

      // If the link is internal, just return the ref.
      if (link.side = $ff) then
      begin
        nextRef := link.ref;
        break;
      end;

      // If the link is at tile boundary,

      // Check if the link spans the whole edge, and accept.
      if (link.bmin = 0) and (link.bmax = 255) then
      begin
        nextRef := link.ref;
        break;
      end;

      // Check for partial edge links.
      v0 := poly.verts[link.edge];
      v1 := poly.verts[(link.edge+1) mod poly.vertCount];
      left := @tile.verts[v0*3];
      right := @tile.verts[v1*3];

      // Check that the intersection lies inside the link portal.
      if (link.side = 0) or (link.side = 4) then
      begin
        // Calculate link size.
        s := 1.0/255.0;
        lmin := left[2] + (right[2] - left[2])*(link.bmin*s);
        lmax := left[2] + (right[2] - left[2])*(link.bmax*s);
        if (lmin > lmax) then dtSwap(lmin, lmax);

        // Find Z intersection.
        z := startPos[2] + (endPos[2]-startPos[2])*tmax;
        if (z >= lmin) and (z <= lmax) then
        begin
          nextRef := link.ref;
          break;
        end;
      end
      else if (link.side = 2) or (link.side = 6) then
      begin
        // Calculate link size.
        s := 1.0/255.0;
        lmin := left[0] + (right[0] - left[0])*(link.bmin*s);
        lmax := left[0] + (right[0] - left[0])*(link.bmax*s);
        if (lmin > lmax) then dtSwap(lmin, lmax);

        // Find X intersection.
        x := startPos[0] + (endPos[0]-startPos[0])*tmax;
        if (x >= lmin) and (x <= lmax) then
        begin
          nextRef := link.ref;
          break;
        end;
      end;

      i := tile.links[i].next;
    end;

    // add the cost
    if (options and Byte(DT_RAYCAST_USE_COSTS)) <> 0 then
    begin
      // compute the intersection point at the furthest end of the polygon
      // and correct the height (since the raycast moves in 2d)
      dtVcopy(@lastPos[0], @curPos[0]);
      dtVmad(@curPos[0], startPos, @dir[0], hit.t);
      e1 := @verts[segMax*3];
      e2 := @verts[((segMax+1) mod nv)*3];
      dtVsub(@eDir[0], e2, e1);
      dtVsub(@diff[0], @curPos[0], e1);
      if Sqr(eDir[0]) > Sqr(eDir[2]) then s := diff[0] / eDir[0] else s := diff[2] / eDir[2];
      curPos[1] := e1[1] + eDir[1] * s;

      hit.pathCost := hit.pathCost + filter.getCost(@lastPos[0], @curPos[0], prevRef, prevTile, prevPoly, curRef, tile, poly, nextRef, nextTile, nextPoly);
    end;

    if (nextRef = 0) then
    begin
      // No neighbour, we hit a wall.

      // Calculate hit normal.
      a := segMax;
      b := IfThen(segMax+1 < nv, segMax+1, 0);
      va := @verts[a*3];
      vb := @verts[b*3];
      dx := vb[0] - va[0];
      dz := vb[2] - va[2];
      hit.hitNormal[0] := dz;
      hit.hitNormal[1] := 0;
      hit.hitNormal[2] := -dx;
      dtVnormalize(@hit.hitNormal[0]);

      hit.pathCount := n;
      Exit(status);
    end;

    // No hit, advance to neighbour polygon.
    prevRef := curRef;
    curRef := nextRef;
    prevTile := tile;
    tile := nextTile;
    prevPoly := poly;
    poly := nextPoly;
  end;

  hit.pathCount := n;

  Result := status;
end;

/// @par
///
/// At least one result array must be provided.
///
/// The order of the result set is from least to highest cost to reach the polygon.
///
/// A common use case for this method is to perform Dijkstra searches.
/// Candidate polygons are found by searching the graph beginning at the start polygon.
///
/// If a polygon is not found via the graph search, even if it intersects the
/// search circle, it will not be included in the result set. For example:
///
/// polyA is the start polygon.
/// polyB shares an edge with polyA. (Is adjacent.)
/// polyC shares an edge with polyB, but not with polyA
/// Even if the search circle overlaps polyC, it will not be included in the
/// result set unless polyB is also in the set.
///
/// The value of the center point is used as the start position for cost
/// calculations. It is not projected onto the surface of the mesh, so its
/// y-value will effect the costs.
///
/// Intersection tests occur in 2D. All polygons and the search circle are
/// projected onto the xz-plane. So the y-value of the center point does not
/// effect intersection tests.
///
/// If the result arrays are to small to hold the entire result set, they will be
/// filled to capacity.
///
function TdtNavMeshQuery.findPolysAroundCircle(startRef: TdtPolyRef; centerPos: PSingle; radius: Single;
                   filter: TdtQueryFilter;
                   resultRef, resultParent: PdtPolyRef; resultCost: PSingle;
                   resultCount: PInteger; maxResult: Integer): TdtStatus;
var startNode,bestNode,neighbourNode: PdtNode; status: TdtStatus; n: Integer; radiusSqr: Single; bestRef,parentRef,neighbourRef: TdtPolyRef;
bestTile,parentTile,neighbourTile: PdtMeshTile; bestPoly,parentPoly,neighbourPoly: PdtPoly; i: Cardinal; link: PdtLink;
va,vb: array [0..2] of Single; tseg,distSqr,total: Single;
begin
  Assert(m_nav <> nil);
  Assert(m_nodePool <> nil);
  Assert(m_openList <> nil);

  resultCount^ := 0;

  // Validate input
  if (startRef = 0) or (not m_nav.isValidPolyRef(startRef)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  m_nodePool.clear();
  m_openList.clear();

  startNode := m_nodePool.getNode(startRef);
  dtVcopy(@startNode.pos, centerPos);
  startNode.pidx := 0;
  startNode.cost := 0;
  startNode.total := 0;
  startNode.id := startRef;
  startNode.flags := DT_NODE_OPEN;
  m_openList.push(startNode);

  status := DT_SUCCESS;

  n := 0;
  if (n < maxResult) then
  begin
    if (resultRef <> nil) then
      resultRef[n] := startNode.id;
    if (resultParent <> nil) then
      resultParent[n] := 0;
    if (resultCost <> nil) then
      resultCost[n] := 0;
    Inc(n);
  end
  else
  begin
    status := status  or DT_BUFFER_TOO_SMALL;
  end;

  radiusSqr := Sqr(radius);

  while (not m_openList.empty()) do
  begin
    bestNode := m_openList.pop();
    bestNode.flags := bestNode.flags and not DT_NODE_OPEN;
    bestNode.flags := bestNode.flags or DT_NODE_CLOSED;

    // Get poly and tile.
    // The API input has been cheked already, skip checking internal data.
    bestRef := bestNode.id;
    bestTile := nil;
    bestPoly := nil;
    m_nav.getTileAndPolyByRefUnsafe(bestRef, @bestTile, @bestPoly);

    // Get parent poly and tile.
    parentRef := 0;
    parentTile := nil;
    parentPoly := nil;
    if (bestNode.pidx <> 0) then
      parentRef := m_nodePool.getNodeAtIdx(bestNode.pidx).id;
    if (parentRef <> 0) then
      m_nav.getTileAndPolyByRefUnsafe(parentRef, @parentTile, @parentPoly);

    i := bestPoly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      link := @bestTile.links[i];
      neighbourRef := link.ref;
      // Skip invalid neighbours and do not follow back to parent.
      if (neighbourRef = 0) or (neighbourRef = parentRef) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Expand to neighbour
      neighbourTile := nil;
      neighbourPoly := nil;
      m_nav.getTileAndPolyByRefUnsafe(neighbourRef, @neighbourTile, @neighbourPoly);

      // Do not advance if the polygon is excluded by the filter.
      if (not filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Find edge and calc distance to the edge.
      if (getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, @va[0], @vb[0]) = 0) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // If the circle is not touching the next polygon, skip it.
      distSqr := dtDistancePtSegSqr2D(centerPos, @va[0], @vb[0], @tseg);
      if (distSqr > radiusSqr) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      neighbourNode := m_nodePool.getNode(neighbourRef);
      if (neighbourNode = nil) then
      begin
        status := status or DT_OUT_OF_NODES;
        i := bestTile.links[i].next;
        continue;
      end;

      if (neighbourNode.flags and DT_NODE_CLOSED) <> 0 then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Cost
      if (neighbourNode.flags = 0) then
        dtVlerp(@neighbourNode.pos, @va[0], @vb[0], 0.5);

      total := bestNode.total + dtVdist(@bestNode.pos, @neighbourNode.pos);

      // The node is already in open list and the new result is worse, skip.
      if ((neighbourNode.flags and DT_NODE_OPEN) <> 0) and (total >= neighbourNode.total) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      neighbourNode.id := neighbourRef;
      neighbourNode.flags := (neighbourNode.flags and not DT_NODE_CLOSED);
      neighbourNode.pidx := m_nodePool.getNodeIdx(bestNode);
      neighbourNode.total := total;

      if (neighbourNode.flags and DT_NODE_OPEN) <> 0 then
      begin
        m_openList.modify(neighbourNode);
      end
      else
      begin
        if (n < maxResult) then
        begin
          if (resultRef <> nil) then
            resultRef[n] := neighbourNode.id;
          if (resultParent <> nil) then
            resultParent[n] := m_nodePool.getNodeAtIdx(neighbourNode.pidx).id;
          if (resultCost <> nil) then
            resultCost[n] := neighbourNode.total;
          Inc(n);
        end
        else
        begin
          status := status or DT_BUFFER_TOO_SMALL;
        end;
        neighbourNode.flags := DT_NODE_OPEN;
        m_openList.push(neighbourNode);
      end;

      i := bestTile.links[i].next;
    end;
  end;

  resultCount^ := n;

  Result := status;
end;

/// @par
///
/// The order of the result set is from least to highest cost.
///
/// At least one result array must be provided.
///
/// A common use case for this method is to perform Dijkstra searches.
/// Candidate polygons are found by searching the graph beginning at the start
/// polygon.
///
/// The same intersection test restrictions that apply to findPolysAroundCircle()
/// method apply to this method.
///
/// The 3D centroid of the search polygon is used as the start position for cost
/// calculations.
///
/// Intersection tests occur in 2D. All polygons are projected onto the
/// xz-plane. So the y-values of the vertices do not effect intersection tests.
///
/// If the result arrays are is too small to hold the entire result set, they will
/// be filled to capacity.
///
function TdtNavMeshQuery.findPolysAroundShape(startRef: TdtPolyRef; verts: PSingle; nverts: Integer;
                  filter: TdtQueryFilter;
                  resultRef, resultParent: PdtPolyRef; resultCost: PSingle;
                  resultCount: PInteger; maxResult: Integer): TdtStatus;
var centerPos: array [0..2] of Single; i: Cardinal; startNode,bestNode,neighbourNode: PdtNode; status: TdtStatus; n: Integer;
bestRef,parentRef,neighbourRef: TdtPolyRef; bestTile,parentTile,neighbourTile: PdtMeshTile; bestPoly,parentPoly,neighbourPoly: PdtPoly;
link: PdtLink; va,vb: array [0..2] of Single; tmin,tmax,distSqr,total: Single; segMin,segMax: Integer;
begin
  Assert(m_nav <> nil);
  Assert(m_nodePool <> nil);
  Assert(m_openList <> nil);

  resultCount^ := 0;

  // Validate input
  if (startRef = 0) or (not m_nav.isValidPolyRef(startRef)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  m_nodePool.clear();
  m_openList.clear();

  centerPos[0] := 0; centerPos[1] := 0; centerPos[2] := 0;
  for i := 0 to nverts - 1 do
    dtVadd(@centerPos[0],@centerPos[0],@verts[i*3]);
  dtVscale(@centerPos[0],@centerPos[0],1.0/nverts);

  startNode := m_nodePool.getNode(startRef);
  dtVcopy(@startNode.pos, @centerPos[0]);
  startNode.pidx := 0;
  startNode.cost := 0;
  startNode.total := 0;
  startNode.id := startRef;
  startNode.flags := DT_NODE_OPEN;
  m_openList.push(startNode);

  status := DT_SUCCESS;

  n := 0;
  if (n < maxResult) then
  begin
    if (resultRef <> nil) then
      resultRef[n] := startNode.id;
    if (resultParent <> nil) then
      resultParent[n] := 0;
    if (resultCost <> nil) then
      resultCost[n] := 0;
    Inc(n);
  end
  else
  begin
    status := status or DT_BUFFER_TOO_SMALL;
  end;

  while (not m_openList.empty()) do
  begin
    bestNode := m_openList.pop();
    bestNode.flags := bestNode.flags and not DT_NODE_OPEN;
    bestNode.flags := bestNode.flags or DT_NODE_CLOSED;

    // Get poly and tile.
    // The API input has been cheked already, skip checking internal data.
    bestRef := bestNode.id;
    bestTile := nil;
    bestPoly := nil;
    m_nav.getTileAndPolyByRefUnsafe(bestRef, @bestTile, @bestPoly);

    // Get parent poly and tile.
    parentRef := 0;
    parentTile := nil;
    parentPoly := nil;
    if (bestNode.pidx <> 0) then
      parentRef := m_nodePool.getNodeAtIdx(bestNode.pidx).id;
    if (parentRef <> 0) then
      m_nav.getTileAndPolyByRefUnsafe(parentRef, @parentTile, @parentPoly);

    i := bestPoly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      link := @bestTile.links[i];
      neighbourRef := link.ref;
      // Skip invalid neighbours and do not follow back to parent.
      if (neighbourRef = 0) or (neighbourRef = parentRef) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Expand to neighbour
      neighbourTile := nil;
      neighbourPoly := nil;
      m_nav.getTileAndPolyByRefUnsafe(neighbourRef, @neighbourTile, @neighbourPoly);

      // Do not advance if the polygon is excluded by the filter.
      if (not filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Find edge and calc distance to the edge.
      if (getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, @va[0], @vb[0]) = 0) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // If the poly is not touching the edge to the next polygon, skip the connection it.
      if (not dtIntersectSegmentPoly2D(@va[0], @vb[0], verts, nverts, @tmin, @tmax, @segMin, @segMax)) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;
      if (tmin > 1.0) or (tmax < 0.0) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      neighbourNode := m_nodePool.getNode(neighbourRef);
      if (neighbourNode = nil) then
      begin
        status := status or DT_OUT_OF_NODES;
        i := bestTile.links[i].next;
        continue;
      end;

      if (neighbourNode.flags and DT_NODE_CLOSED) <> 0 then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Cost
      if (neighbourNode.flags = 0) then
        dtVlerp(@neighbourNode.pos, @va[0], @vb[0], 0.5);

      total := bestNode.total + dtVdist(@bestNode.pos, @neighbourNode.pos);

      // The node is already in open list and the new result is worse, skip.
      if ((neighbourNode.flags and DT_NODE_OPEN) <> 0) and (total >= neighbourNode.total) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      neighbourNode.id := neighbourRef;
      neighbourNode.flags := (neighbourNode.flags and not DT_NODE_CLOSED);
      neighbourNode.pidx := m_nodePool.getNodeIdx(bestNode);
      neighbourNode.total := total;

      if (neighbourNode.flags and DT_NODE_OPEN) <> 0 then
      begin
        m_openList.modify(neighbourNode);
      end
      else
      begin
        if (n < maxResult) then
        begin
          if (resultRef <> nil) then
            resultRef[n] := neighbourNode.id;
          if (resultParent <> nil) then
            resultParent[n] := m_nodePool.getNodeAtIdx(neighbourNode.pidx).id;
          if (resultCost <> nil) then
            resultCost[n] := neighbourNode.total;
          Inc(n);
        end
        else
        begin
          status := status or DT_BUFFER_TOO_SMALL;
        end;
        neighbourNode.flags := DT_NODE_OPEN;
        m_openList.push(neighbourNode);
      end;

      i := bestTile.links[i].next;
    end;
  end;

  resultCount^ := n;

  Result := status;
end;

/// @par
///
/// This method is optimized for a small search radius and small number of result
/// polygons.
///
/// Candidate polygons are found by searching the navigation graph beginning at
/// the start polygon.
///
/// The same intersection test restrictions that apply to the findPolysAroundCircle
/// mehtod applies to this method.
///
/// The value of the center point is used as the start point for cost calculations.
/// It is not projected onto the surface of the mesh, so its y-value will effect
/// the costs.
///
/// Intersection tests occur in 2D. All polygons and the search circle are
/// projected onto the xz-plane. So the y-value of the center point does not
/// effect intersection tests.
///
/// If the result arrays are is too small to hold the entire result set, they will
/// be filled to capacity.
///
function TdtNavMeshQuery.findLocalNeighbourhood(startRef: TdtPolyRef; centerPos: PSingle; radius: Single;
                  filter: TdtQueryFilter;
                  resultRef, resultParent: PdtPolyRef;
                  resultCount: PInteger; maxResult: Integer): TdtStatus;
const MAX_STACK = 48;
var stack: array [0..MAX_STACK-1] of PdtNode; i, k: Cardinal; nstack,n,j,i0,k0: Integer; startNode, curNode, neighbourNode: PdtNode; radiusSqr: Single;
pa, pb: array [0..DT_VERTS_PER_POLYGON*3-1] of Single; status: TdtStatus; curRef, neighbourRef, pastRef: TdtPolyRef;
curTile, neighbourTile, pastTile: PdtMeshTile; curPoly, neighbourPoly, pastPoly: PdtPoly;
link: PdtLink; va,vb: array [0..2] of Single; tseg, distSqr: Single; npa, npb: Integer; overlap, connected: Boolean;
begin
  Assert(m_nav <> nil);
  Assert(m_tinyNodePool <> nil);

  resultCount^ := 0;

  // Validate input
  if (startRef = 0) or (not m_nav.isValidPolyRef(startRef)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  nstack := 0;

  m_tinyNodePool.clear();

  startNode := m_tinyNodePool.getNode(startRef);
  startNode.pidx := 0;
  startNode.id := startRef;
  startNode.flags := DT_NODE_CLOSED;
  stack[nstack] := startNode;
  Inc(nstack);

  radiusSqr := Sqr(radius);

  status := DT_SUCCESS;

  n := 0;
  if (n < maxResult) then
  begin
    resultRef[n] := startNode.id;
    if (resultParent <> nil) then
      resultParent[n] := 0;
    Inc(n);
  end
  else
  begin
    status := status or DT_BUFFER_TOO_SMALL;
  end;

  while (nstack <> 0) do
  begin
    // Pop front.
    curNode := stack[0];
    for i0 := 0 to nstack-1 - 1 do
      stack[i0] := stack[i0+1];
    Dec(nstack);

    // Get poly and tile.
    // The API input has been cheked already, skip checking internal data.
    curRef := curNode.id;
    curTile := nil;
    curPoly := nil;
    m_nav.getTileAndPolyByRefUnsafe(curRef, @curTile, @curPoly);

    i := curPoly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      link := @curTile.links[i];
      neighbourRef := link.ref;
      // Skip invalid neighbours.
      if (neighbourRef = 0) then
      begin
        i := curTile.links[i].next;
        continue;
      end;

      // Skip if cannot alloca more nodes.
      neighbourNode := m_tinyNodePool.getNode(neighbourRef);
      if (neighbourNode = nil) then
      begin
        i := curTile.links[i].next;
        continue;
      end;
      // Skip visited.
      if (neighbourNode.flags and DT_NODE_CLOSED) <> 0 then
      begin
        i := curTile.links[i].next;
        continue;
      end;

      // Expand to neighbour
      neighbourTile := nil;
      neighbourPoly := nil;
      m_nav.getTileAndPolyByRefUnsafe(neighbourRef, @neighbourTile, @neighbourPoly);

      // Skip off-mesh connections.
      if (neighbourPoly.getType() = DT_POLYTYPE_OFFMESH_CONNECTION) then
      begin
        i := curTile.links[i].next;
        continue;
      end;

      // Do not advance if the polygon is excluded by the filter.
      if (not filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) then
      begin
        i := curTile.links[i].next;
        continue;
      end;

      // Find edge and calc distance to the edge.
      if (getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly, neighbourTile, @va[0], @vb[0]) = 0) then
      begin
        i := curTile.links[i].next;
        continue;
      end;

      // If the circle is not touching the next polygon, skip it.
      distSqr := dtDistancePtSegSqr2D(centerPos, @va[0], @vb[0], @tseg);
      if (distSqr > radiusSqr) then
      begin
        i := curTile.links[i].next;
        continue;
      end;

      // Mark node visited, this is done before the overlap test so that
      // we will not visit the poly again if the test fails.
      neighbourNode.flags := neighbourNode.flags or DT_NODE_CLOSED;
      neighbourNode.pidx := m_tinyNodePool.getNodeIdx(curNode);

      // Check that the polygon does not collide with existing polygons.

      // Collect vertices of the neighbour poly.
      npa := neighbourPoly.vertCount;
      for k := 0 to npa - 1 do
        dtVcopy(@pa[k*3], @neighbourTile.verts[neighbourPoly.verts[k]*3]);

      overlap := false;
      for j := 0 to n - 1 do
      begin
        pastRef := resultRef[j];

        // Connected polys do not overlap.
        connected := false;
        k := curPoly.firstLink;
        while (k <> DT_NULL_LINK) do
        begin
          if (curTile.links[k].ref = pastRef) then
          begin
            connected := true;
            break;
          end;

          k := curTile.links[k].next;
        end;
        if (connected) then
          continue;

        // Potentially overlapping.
        pastTile := nil;
        pastPoly := nil;
        m_nav.getTileAndPolyByRefUnsafe(pastRef, @pastTile, @pastPoly);

        // Get vertices and test overlap
        npb := pastPoly.vertCount;
        for k0 := 0 to npb - 1 do
          dtVcopy(@pb[k0*3], @pastTile.verts[pastPoly.verts[k0]*3]);

        if (dtOverlapPolyPoly2D(@pa[0],npa, @pb[0],npb)) then
        begin
          overlap := true;
          break;
        end;
      end;
      if (overlap) then
      begin
        i := curTile.links[i].next;
        continue;
      end;

      // This poly is fine, store and advance to the poly.
      if (n < maxResult) then
      begin
        resultRef[n] := neighbourRef;
        if (resultParent <> nil) then
          resultParent[n] := curRef;
        Inc(n);
      end
      else
      begin
        status := status or DT_BUFFER_TOO_SMALL;
      end;

      if (nstack < MAX_STACK) then
      begin
        stack[nstack] := neighbourNode;
        Inc(nstack);
      end;

      i := curTile.links[i].next;
    end;
  end;

  resultCount^ := n;

  Result := status;
end;

type
  PdtSegInterval = ^TdtSegInterval;
  TdtSegInterval = record
    ref: TdtPolyRef;
    tmin, tmax: SmallInt;
  end;

procedure insertInterval(ints: PdtSegInterval; nints: PInteger; maxInts: Integer;
               tmin, tmax: SmallInt; ref: TdtPolyRef);
var idx: Integer;
begin
  if (nints^+1 > maxInts) then Exit;
  // Find insertion point.
  idx := 0;
  while (idx < nints^) do
  begin
    if (tmax <= ints[idx].tmin) then
      break;
    Inc(idx);
  end;
  // Move current results.
  if (nints^-idx) <> 0 then
    Move(ints[idx], ints[idx+1], sizeof(TdtSegInterval)*(nints^-idx));
  // Store
  ints[idx].ref := ref;
  ints[idx].tmin := tmin;
  ints[idx].tmax := tmax;
  Inc(nints^);
end;

/// @par
///
/// If the @p segmentRefs parameter is provided, then all polygon segments will be returned.
/// Otherwise only the wall segments are returned.
///
/// A segment that is normally a portal will be included in the result set as a
/// wall if the @p filter results in the neighbor polygon becoomming impassable.
///
/// The @p segmentVerts and @p segmentRefs buffers should normally be sized for the
/// maximum segments per polygon of the source navigation mesh.
///
function TdtNavMeshQuery.getPolyWallSegments(ref: TdtPolyRef; filter: TdtQueryFilter;
                 segmentVerts: PSingle; segmentRefs: PdtPolyRef; segmentCount: PInteger;
                 maxSegments: Integer): TdtStatus;
const MAX_INTERVAL = 16;
var tile,neiTile: PdtMeshTile; poly,neiPoly: PdtPoly; n,nints: Integer; ints: array [0..MAX_INTERVAL-1] of TdtSegInterval; storePortals: Boolean;
status: TdtStatus; i,j: Integer; k: Cardinal; link: PdtLink; neiRef: TdtPolyRef; idx: Cardinal; vj,vi,seg: PSingle; tmin,tmax: Single;
imin,imax: Integer;
begin
  //dtAssert(m_nav);

  segmentCount^ := 0;

  tile := nil;
  poly := nil;
  if (dtStatusFailed(m_nav.getTileAndPolyByRef(ref, @tile, @poly))) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  n := 0;

  storePortals := segmentRefs <> nil;

  status := DT_SUCCESS;

  i := 0;
  j := poly.vertCount-1;
  while (i < poly.vertCount) do
  begin
    // Skip non-solid edges.
    nints := 0;
    if (poly.neis[j] and DT_EXT_LINK) <> 0 then
    begin
      // Tile border.
      k := poly.firstLink;
      while (k <> DT_NULL_LINK) do
      begin
        link := @tile.links[k];
        if (link.edge = j) then
        begin
          if (link.ref <> 0) then
          begin
            neiTile := nil;
            neiPoly := nil;
            m_nav.getTileAndPolyByRefUnsafe(link.ref, @neiTile, @neiPoly);
            if (filter.passFilter(link.ref, neiTile, neiPoly)) then
            begin
              insertInterval(@ints[0], @nints, MAX_INTERVAL, link.bmin, link.bmax, link.ref);
            end;
          end;
        end;
        k := tile.links[k].next;
      end;
    end
    else
    begin
      // Internal edge
      neiRef := 0;
      if (poly.neis[j] <> 0) then
      begin
        idx := (poly.neis[j]-1);
        neiRef := m_nav.getPolyRefBase(tile) or idx;
        if (not filter.passFilter(neiRef, tile, @tile.polys[idx])) then
          neiRef := 0;
      end;

      // If the edge leads to another polygon and portals are not stored, skip.
      if (neiRef <> 0) and (not storePortals) then
      begin
        j := i;
        Inc(i);
        continue;
      end;

      if (n < maxSegments) then
      begin
        vj := @tile.verts[poly.verts[j]*3];
        vi := @tile.verts[poly.verts[i]*3];
        seg := @segmentVerts[n*6];
        dtVcopy(seg+0, vj);
        dtVcopy(seg+3, vi);
        if (segmentRefs <> nil) then
          segmentRefs[n] := neiRef;
        Inc(n);
      end
      else
      begin
        status := status or DT_BUFFER_TOO_SMALL;
      end;

      j := i;
      Inc(i);
      continue;
    end;

    // Add sentinels
    insertInterval(@ints[0], @nints, MAX_INTERVAL, -1, 0, 0);
    insertInterval(@ints[0], @nints, MAX_INTERVAL, 255, 256, 0);

    // Store segments.
    vj := @tile.verts[poly.verts[j]*3];
    vi := @tile.verts[poly.verts[i]*3];
    for k := 1 to nints - 1 do
    begin
      // Portal segment.
      if storePortals and (ints[k].ref <> 0) then
      begin
        tmin := ints[k].tmin/255.0;
        tmax := ints[k].tmax/255.0;
        if (n < maxSegments) then
        begin
          seg := @segmentVerts[n*6];
          dtVlerp(seg+0, vj,vi, tmin);
          dtVlerp(seg+3, vj,vi, tmax);
          if (segmentRefs <> nil) then
            segmentRefs[n] := ints[k].ref;
          Inc(n);
        end
        else
        begin
          status := status or DT_BUFFER_TOO_SMALL;
        end;
      end;

      // Wall segment.
      imin := ints[k-1].tmax;
      imax := ints[k].tmin;
      if (imin <> imax) then
      begin
        tmin := imin/255.0;
        tmax := imax/255.0;
        if (n < maxSegments) then
        begin
          seg := @segmentVerts[n*6];
          dtVlerp(seg+0, vj,vi, tmin);
          dtVlerp(seg+3, vj,vi, tmax);
          if (segmentRefs <> nil) then
            segmentRefs[n] := 0;
          Inc(n);
        end
        else
        begin
          status := status or DT_BUFFER_TOO_SMALL;
        end;
      end;
    end;

    j := i;
    Inc(i);
  end;

  segmentCount^ := n;

  Result := status;
end;

/// @par
///
/// @p hitPos is not adjusted using the height detail data.
///
/// @p hitDist will equal the search radius if there is no wall within the
/// radius. In this case the values of @p hitPos and @p hitNormal are
/// undefined.
///
/// The normal will become unpredicable if @p hitDist is a very small number.
///
function TdtNavMeshQuery.findDistanceToWall(startRef: TdtPolyRef; centerPos: PSingle; maxRadius: Single;
                filter: TdtQueryFilter;
                hitDist, hitPos, hitNormal: PSingle): TdtStatus;
var startNode,bestNode,neighbourNode: PdtNode; radiusSqr: Single; status: TdtStatus; bestRef,parentRef: TdtPolyRef;
bestTile,parentTile,neiTile,neighbourTile: PdtMeshTile; bestPoly,parentPoly,neiPoly,neighbourPoly: PdtPoly; i,k: Cardinal; j: Integer;
solid: Boolean; link: PdtLink; idx: Cardinal; ref,neighbourRef: TdtPolyRef; vj,vi,va,vb: PSingle; tseg,distSqr,total: Single;
begin
  //dtAssert(m_nav);
  //dtAssert(m_nodePool);
  //dtAssert(m_openList);

  // Validate input
  if (startRef = 0) or (not m_nav.isValidPolyRef(startRef)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  m_nodePool.clear();
  m_openList.clear();

  startNode := m_nodePool.getNode(startRef);
  dtVcopy(@startNode.pos, centerPos);
  startNode.pidx := 0;
  startNode.cost := 0;
  startNode.total := 0;
  startNode.id := startRef;
  startNode.flags := DT_NODE_OPEN;
  m_openList.push(startNode);

  radiusSqr := Sqr(maxRadius);

  status := DT_SUCCESS;

  while (not m_openList.empty()) do
  begin
    bestNode := m_openList.pop();
    bestNode.flags := bestNode.flags and not DT_NODE_OPEN;
    bestNode.flags := bestNode.flags or DT_NODE_CLOSED;

    // Get poly and tile.
    // The API input has been cheked already, skip checking internal data.
    bestRef := bestNode.id;
    bestTile := nil;
    bestPoly := nil;
    m_nav.getTileAndPolyByRefUnsafe(bestRef, @bestTile, @bestPoly);

    // Get parent poly and tile.
    parentRef := 0;
    parentTile := nil;
    parentPoly := nil;
    if (bestNode.pidx <> 0) then
      parentRef := m_nodePool.getNodeAtIdx(bestNode.pidx).id;
    if (parentRef <> 0) then
      m_nav.getTileAndPolyByRefUnsafe(parentRef, @parentTile, @parentPoly);

    // Hit test walls.
    i := 0;
    j := bestPoly.vertCount-1;
    while (i < bestPoly.vertCount) do
    begin
      // Skip non-solid edges.
      if (bestPoly.neis[j] and DT_EXT_LINK) <> 0 then
      begin
        // Tile border.
        solid := true;
        k := bestPoly.firstLink;
        while (k <> DT_NULL_LINK) do
        begin
          link := @bestTile.links[k];
          if (link.edge = j) then
          begin
            if (link.ref <> 0) then
            begin
              neiTile := nil;
              neiPoly := nil;
              m_nav.getTileAndPolyByRefUnsafe(link.ref, @neiTile, @neiPoly);
              if (filter.passFilter(link.ref, neiTile, neiPoly)) then
                solid := false;
            end;
            break;
          end;
          k := bestTile.links[k].next;
        end;
        if (not solid) then begin j := i; Inc(i); continue; end;
      end
      else if (bestPoly.neis[j] <> 0) then
      begin
        // Internal edge
        idx := (bestPoly.neis[j]-1);
        ref := m_nav.getPolyRefBase(bestTile) or idx;
        if (filter.passFilter(ref, bestTile, @bestTile.polys[idx])) then
        begin
          j := i;
          Inc(i);
          continue;
        end;
      end;

      // Calc distance to the edge.
      vj := @bestTile.verts[bestPoly.verts[j]*3];
      vi := @bestTile.verts[bestPoly.verts[i]*3];
      distSqr := dtDistancePtSegSqr2D(centerPos, vj, vi, @tseg);

      // Edge is too far, skip.
      if (distSqr > radiusSqr) then
      begin
        j := i;
        Inc(i);
        continue;
      end;

      // Hit wall, update radius.
      radiusSqr := distSqr;
      // Calculate hit pos.
      hitPos[0] := vj[0] + (vi[0] - vj[0])*tseg;
      hitPos[1] := vj[1] + (vi[1] - vj[1])*tseg;
      hitPos[2] := vj[2] + (vi[2] - vj[2])*tseg;

      j := i;
      Inc(i);
    end;

    i := bestPoly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      link := @bestTile.links[i];
      neighbourRef := link.ref;
      // Skip invalid neighbours and do not follow back to parent.
      if (neighbourRef = 0) or (neighbourRef = parentRef) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Expand to neighbour.
      neighbourTile := nil;
      neighbourPoly := nil;
      m_nav.getTileAndPolyByRefUnsafe(neighbourRef, @neighbourTile, @neighbourPoly);

      // Skip off-mesh connections.
      if (neighbourPoly.getType = DT_POLYTYPE_OFFMESH_CONNECTION) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Calc distance to the edge.
      va := @bestTile.verts[bestPoly.verts[link.edge]*3];
      vb := @bestTile.verts[bestPoly.verts[(link.edge+1) mod bestPoly.vertCount]*3];
      distSqr := dtDistancePtSegSqr2D(centerPos, va, vb, @tseg);

      // If the circle is not touching the next polygon, skip it.
      if (distSqr > radiusSqr) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      if (not filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      neighbourNode := m_nodePool.getNode(neighbourRef);
      if (neighbourNode = nil) then
      begin
        status := status or DT_OUT_OF_NODES;
        i := bestTile.links[i].next;
        continue;
      end;

      if (neighbourNode.flags and DT_NODE_CLOSED) <> 0 then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      // Cost
      if (neighbourNode.flags = 0) then
      begin
        getEdgeMidPoint(bestRef, bestPoly, bestTile,
                neighbourRef, neighbourPoly, neighbourTile, @neighbourNode.pos);
      end;

      total := bestNode.total + dtVdist(@bestNode.pos, @neighbourNode.pos);

      // The node is already in open list and the new result is worse, skip.
      if ((neighbourNode.flags and DT_NODE_OPEN) <> 0) and (total >= neighbourNode.total) then
      begin
        i := bestTile.links[i].next;
        continue;
      end;

      neighbourNode.id := neighbourRef;
      neighbourNode.flags := (neighbourNode.flags and not DT_NODE_CLOSED);
      neighbourNode.pidx := m_nodePool.getNodeIdx(bestNode);
      neighbourNode.total := total;

      if (neighbourNode.flags and DT_NODE_OPEN) <> 0 then
      begin
        m_openList.modify(neighbourNode);
      end
      else
      begin
        neighbourNode.flags := neighbourNode.flags or DT_NODE_OPEN;
        m_openList.push(neighbourNode);
      end;

      i := bestTile.links[i].next;
    end;
  end;

  // Calc hit normal.
  dtVsub(hitNormal, centerPos, hitPos);
  dtVnormalize(hitNormal);

  hitDist^ := Sqrt(radiusSqr);

  Result := status;
end;

function TdtNavMeshQuery.isValidPolyRef(ref: TdtPolyRef; filter: TdtQueryFilter): Boolean;
var tile: PdtMeshTile; poly: PdtPoly; status: TdtStatus;
begin
  tile := nil;
  poly := nil;
  status := m_nav.getTileAndPolyByRef(ref, @tile, @poly);
  // If cannot get polygon, assume it does not exists and boundary is invalid.
  if (dtStatusFailed(status)) then
    Exit(false);
  // If cannot pass filter, assume flags has changed and boundary is invalid.
  if (not filter.passFilter(ref, tile, poly)) then
    Exit(false);
  Result := true;
end;

/// @par
///
/// The closed list is the list of polygons that were fully evaluated during
/// the last navigation graph search. (A* or Dijkstra)
///
function TdtNavMeshQuery.isInClosedList(ref: TdtPolyRef): Boolean;
var nodes: array [0..DT_MAX_STATES_PER_NODE-1] of PdtNode; n,i: Integer;
begin
  if (m_nodePool = nil) then Exit(false);

  n := m_nodePool.findNodes(ref, nodes, DT_MAX_STATES_PER_NODE);

  for i := 0 to n - 1 do
  begin
    if (nodes[i].flags and DT_NODE_CLOSED) <> 0 then
      Exit(true);
  end;

  Result := false;
end;


end.
