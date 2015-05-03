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

unit RN_DetourNavMesh;
interface
uses
  Classes, Math, RN_DetourNode, RN_DetourNavMeshHelper, RN_DetourCommon, RN_DetourStatus;


  /// The maximum number of vertices per navigation polygon.
  /// @ingroup detour
  const DT_VERTS_PER_POLYGON = 6;

  /// @{
  /// @name Tile Serialization Constants
  /// These constants are used to detect whether a navigation tile's data
  /// and state format is compatible with the current build.
  ///

  /// A magic number used to detect compatibility of navigation tile data.
  const DT_NAVMESH_MAGIC = Ord('D') shl 24 or Ord('N') shl 16 or Ord('A') shl 8 or Ord('V');

  /// A version number used to detect compatibility of navigation tile data.
  const DT_NAVMESH_VERSION = 7;

  /// A magic number used to detect the compatibility of navigation tile states.
  const DT_NAVMESH_STATE_MAGIC = Ord('D') shl 24 or Ord('N') shl 16 or Ord('M') shl 8 or Ord('S');

  /// A version number used to detect compatibility of navigation tile states.
  const DT_NAVMESH_STATE_VERSION = 1;

  /// @}

  /// A flag that indicates that an entity links to an external entity.
  /// (E.g. A polygon edge is a portal that links to another polygon.)
  const DT_EXT_LINK = $8000;

  /// A value that indicates the entity does not link to anything.
  const DT_NULL_LINK = $ffffffff;

  /// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
  const DT_OFFMESH_CON_BIDIR = 1;

  /// The maximum number of user defined area ids.
  /// @ingroup detour
  const DT_MAX_AREAS = 64;

  /// Tile flags used for various functions and fields.
  /// For an example, see dtNavMesh::addTile().
  //TdtTileFlags =
    /// The navigation mesh owns the tile memory and is responsible for freeing it.
    DT_TILE_FREE_DATA = $01;

  type
  /// Vertex flags returned by dtNavMeshQuery::findStraightPath.
  TdtStraightPathFlags =
  (
    DT_STRAIGHTPATH_START = $01,        ///< The vertex is the start position in the path.
    DT_STRAIGHTPATH_END = $02,          ///< The vertex is the end position in the path.
    DT_STRAIGHTPATH_OFFMESH_CONNECTION = $04  ///< The vertex is the start of an off-mesh connection.
  );

  /// Options for dtNavMeshQuery::findStraightPath.
  TdtStraightPathOptions =
  (
    DT_STRAIGHTPATH_AREA_CROSSINGS = $01,  ///< Add a vertex at every polygon edge crossing where area changes.
    DT_STRAIGHTPATH_ALL_CROSSINGS = $02  ///< Add a vertex at every polygon edge crossing.
  );


  /// Options for dtNavMeshQuery::findPath
  TdtFindPathOptions =
  (
    DT_FINDPATH_LOW_QUALITY_FAR = $01,    ///< [provisional] trade quality for performance far from the origin. The idea is that by then a new query will be issued
    DT_FINDPATH_ANY_ANGLE  = $02      ///< use raycasts during pathfind to "shortcut" (raycast still consider costs)
  );

  /// Options for dtNavMeshQuery::raycast
  TdtRaycastOptions =
  (
    DT_RAYCAST_USE_COSTS = $01    ///< Raycast should calculate movement cost along the ray and fill RaycastHit::cost
  );


  /// Limit raycasting during any angle pahfinding
  /// The limit is given as a multiple of the character radius
  const DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0;

  /// Flags representing the type of a navigation mesh polygon.
  //dtPolyTypes =

    /// The polygon is a standard convex polygon that is part of the surface of the mesh.
    DT_POLYTYPE_GROUND = 0;
    /// The polygon is an off-mesh connection consisting of two vertices.
    DT_POLYTYPE_OFFMESH_CONNECTION = 1;


  type
  /// Defines a polyogn within a dtMeshTile object.
  /// @ingroup detour
  PPdtPoly = ^PdtPoly;
  PdtPoly = ^TdtPoly;
  TdtPoly = record

    /// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.)
    firstLink: Cardinal;

    /// The indices of the polygon's vertices.
    /// The actual vertices are located in dtMeshTile::verts.
    verts: array [0..DT_VERTS_PER_POLYGON-1] of Word;

    /// Packed data representing neighbor polygons references and flags for each edge.
    neis: array [0..DT_VERTS_PER_POLYGON-1] of Word;

    /// The user defined polygon flags.
    flags: Word;

    /// The number of vertices in the polygon.
    vertCount: Byte;

    /// The bit packed area id and polygon type.
    /// @note Use the structure's set and get methods to acess this value.
    areaAndtype: Byte;

    /// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
    procedure setArea(a: Byte);

    /// Sets the polygon type. (See: #dtPolyTypes.)
    procedure setType(t: Byte);

    /// Gets the user defined area id.
    function getArea(): Byte;

    /// Gets the polygon type. (See: #dtPolyTypes)
    function getType(): Byte;
  end;

  /// Defines the location of detail sub-mesh data within a dtMeshTile.
  PdtPolyDetail = ^TdtPolyDetail;
  TdtPolyDetail = record
    vertBase: Cardinal;      ///< The offset of the vertices in the dtMeshTile::detailVerts array.
    triBase: Cardinal;      ///< The offset of the triangles in the dtMeshTile::detailTris array.
    vertCount: Byte;    ///< The number of vertices in the sub-mesh.
    triCount: Byte;      ///< The number of triangles in the sub-mesh.
  end;

  /// Defines a link between polygons.
  /// @note This structure is rarely if ever used by the end user.
  /// @see dtMeshTile
  PdtLink = ^TdtLink;
  TdtLink = record
    ref: TdtPolyRef;          ///< Neighbour reference. (The neighbor that is linked to.)
    next: Cardinal;        ///< Index of the next link.
    edge: Byte;        ///< Index of the polygon edge that owns this link.
    side: Byte;        ///< If a boundary link, defines on which side the link is.
    bmin: Byte;        ///< If a boundary link, defines the minimum sub-edge area.
    bmax: Byte;        ///< If a boundary link, defines the maximum sub-edge area.
  end;

  /// Bounding volume node.
  /// @note This structure is rarely if ever used by the end user.
  /// @see dtMeshTile
  PdtBVNode = ^TdtBVNode;
  TdtBVNode = record
    bmin: array [0..2] of Word;      ///< Minimum bounds of the node's AABB. [(x, y, z)]
    bmax: array [0..2] of Word;      ///< Maximum bounds of the node's AABB. [(x, y, z)]
    i: Integer;              ///< The node's index. (Negative for escape sequence.)
  end;

  /// Defines an navigation mesh off-mesh connection within a dtMeshTile object.
  /// An off-mesh connection is a user defined traversable connection made up to two vertices.
  PdtOffMeshConnection = ^TdtOffMeshConnection;
  TdtOffMeshConnection = record
    /// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
    pos: array [0..5] of Single;

    /// The radius of the endpoints. [Limit: >= 0]
    rad: Single;

    /// The polygon reference of the connection within the tile.
    poly: Word;

    /// Link flags.
    /// @note These are not the connection's user defined flags. Those are assigned via the
    /// connection's dtPoly definition. These are link flags used for internal purposes.
    flags: Byte;

    /// End point side.
    side: Byte;

    /// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
    userId: Cardinal;
  end;

  /// Provides high level information related to a dtMeshTile object.
  /// @ingroup detour
  PdtMeshHeader = ^TdtMeshHeader;
  TdtMeshHeader = record
    magic: Integer;        ///< Tile magic number. (Used to identify the data format.)
    version: Integer;      ///< Tile data format version number.
    x: Integer;          ///< The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
    y: Integer;          ///< The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
    layer: Integer;        ///< The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
    userId: Cardinal;  ///< The user defined id of the tile.
    polyCount: Integer;      ///< The number of polygons in the tile.
    vertCount: Integer;      ///< The number of vertices in the tile.
    maxLinkCount: Integer;    ///< The number of allocated links.
    detailMeshCount: Integer;  ///< The number of sub-meshes in the detail mesh.

    /// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
    detailVertCount: Integer;

    detailTriCount: Integer;      ///< The number of triangles in the detail mesh.
    bvNodeCount: Integer;      ///< The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
    offMeshConCount: Integer;    ///< The number of off-mesh connections.
    offMeshBase: Integer;      ///< The index of the first polygon which is an off-mesh connection.
    walkableHeight: Single;    ///< The height of the agents using the tile.
    walkableRadius: Single;    ///< The radius of the agents using the tile.
    walkableClimb: Single;    ///< The maximum climb height of the agents using the tile.
    bmin: array [0..2] of Single;        ///< The minimum bounds of the tile's AABB. [(x, y, z)]
    bmax: array [0..2] of Single;        ///< The maximum bounds of the tile's AABB. [(x, y, z)]

    /// The bounding volume quantization factor.
    bvQuantFactor: Single;
  end;

  /// Defines a navigation mesh tile.
  /// @ingroup detour
  PPdtMeshTile = ^PdtMeshTile;
  PdtMeshTile = ^TdtMeshTile;
  TdtMeshTile = record
    salt: Cardinal;          ///< Counter describing modifications to the tile.

    linksFreeList: Cardinal;      ///< Index to the next free link.
    header: PdtMeshHeader;        ///< The tile header.
    polys: PdtPoly;            ///< The tile polygons. [Size: dtMeshHeader::polyCount]
    verts: PSingle;            ///< The tile vertices. [Size: dtMeshHeader::vertCount]
    links: PdtLink;            ///< The tile links. [Size: dtMeshHeader::maxLinkCount]
    detailMeshes: PdtPolyDetail;      ///< The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]

    /// The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
    detailVerts: PSingle;

    /// The detail mesh's triangles. [(vertA, vertB, vertC) * dtMeshHeader::detailTriCount]
    detailTris: PByte;

    /// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount]
    /// (Will be null if bounding volumes are disabled.)
    bvTree: PdtBVNode;

    offMeshCons: PdtOffMeshConnection;    ///< The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]

    data: PByte;          ///< The tile data. (Not directly accessed under normal situations.)
    dataSize: Integer;              ///< Size of the tile data.
    flags: Integer;                ///< Tile flags. (See: #dtTileFlags)
    next: PdtMeshTile;            ///< The next free tile, or the next tile in the spatial grid.
  end;

  /// Configuration parameters used to define multi-tile navigation meshes.
  /// The values are used to allocate space during the initialization of a navigation mesh.
  /// @see dtNavMesh::init()
  /// @ingroup detour
  PdtNavMeshParams = ^TdtNavMeshParams;
  TdtNavMeshParams = record
    orig: array [0..2] of Single;          ///< The world space origin of the navigation mesh's tile space. [(x, y, z)]
    tileWidth: Single;        ///< The width of each tile. (Along the x-axis.)
    tileHeight: Single;        ///< The height of each tile. (Along the z-axis.)
    maxTiles: Integer;          ///< The maximum number of tiles the navigation mesh can contain.
    maxPolys: Integer;          ///< The maximum number of polygons each tile can contain.
  end;

  /// A navigation mesh based on tiles of convex polygons.
  /// @ingroup detour
  TdtNavMesh = class
  public
    constructor Create();
    destructor Destroy; override;

    /// @{
    /// @name Initialization and Tile Management

    /// Initializes the navigation mesh for tiled use.
    ///  @param[in]  params    Initialization parameters.
    /// @return The status flags for the operation.
    function init(params: PdtNavMeshParams): TdtStatus; overload;

    /// Initializes the navigation mesh for single tile use.
    ///  @param[in]  data    Data of the new tile. (See: #dtCreateNavMeshData)
    ///  @param[in]  dataSize  The data size of the new tile.
    ///  @param[in]  flags    The tile flags. (See: #dtTileFlags)
    /// @return The status flags for the operation.
    ///  @see dtCreateNavMeshData
    function init(data: PByte; dataSize, flags: Integer): TdtStatus; overload;

    /// The navigation mesh initialization params.
    function getParams(): PdtNavMeshParams;

    /// Adds a tile to the navigation mesh.
    ///  @param[in]    data    Data for the new tile mesh. (See: #dtCreateNavMeshData)
    ///  @param[in]    dataSize  Data size of the new tile mesh.
    ///  @param[in]    flags    Tile flags. (See: #dtTileFlags)
    ///  @param[in]    lastRef    The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
    ///  @param[out]  result    The tile reference. (If the tile was succesfully added.) [opt]
    /// @return The status flags for the operation.
    function addTile(data: PByte; dataSize, flags: Integer; lastRef: TdtTileRef; reslt: PdtTileRef): TdtStatus;

    /// Removes the specified tile from the navigation mesh.
    ///  @param[in]    ref      The reference of the tile to remove.
    ///  @param[out]  data    Data associated with deleted tile.
    ///  @param[out]  dataSize  Size of the data associated with deleted tile.
    /// @return The status flags for the operation.
    function removeTile(ref: TdtTileRef; data: PPointer; dataSize: PInteger): TdtStatus;

    /// @}

    /// @{
    /// @name Query Functions

    /// Calculates the tile grid location for the specified world position.
    ///  @param[in]  pos  The world position for the query. [(x, y, z)]
    ///  @param[out]  tx    The tile's x-location. (x, y)
    ///  @param[out]  ty    The tile's y-location. (x, y)
    procedure calcTileLoc(pos: PSingle; tx, ty: PInteger);

    /// Gets the tile at the specified grid location.
    ///  @param[in]  x    The tile's x-location. (x, y, layer)
    ///  @param[in]  y    The tile's y-location. (x, y, layer)
    ///  @param[in]  layer  The tile's layer. (x, y, layer)
    /// @return The tile, or null if the tile does not exist.
    function getTileAt(x, y, layer: Integer): PdtMeshTile;

    /// Gets all tiles at the specified grid location. (All layers.)
    ///  @param[in]    x      The tile's x-location. (x, y)
    ///  @param[in]    y      The tile's y-location. (x, y)
    ///  @param[out]  tiles    A pointer to an array of tiles that will hold the result.
    ///  @param[in]    maxTiles  The maximum tiles the tiles parameter can hold.
    /// @return The number of tiles returned in the tiles array.
    function getTilesAt(x, y: Integer; tiles: PPdtMeshTile; maxTiles: Integer): Integer;

    /// Gets the tile reference for the tile at specified grid location.
    ///  @param[in]  x    The tile's x-location. (x, y, layer)
    ///  @param[in]  y    The tile's y-location. (x, y, layer)
    ///  @param[in]  layer  The tile's layer. (x, y, layer)
    /// @return The tile reference of the tile, or 0 if there is none.
    function getTileRefAt(x, y, layer: Integer): TdtTileRef;

    /// Gets the tile reference for the specified tile.
    ///  @param[in]  tile  The tile.
    /// @return The tile reference of the tile.
    function getTileRef(tile: PdtMeshTile): TdtTileRef;

    /// Gets the tile for the specified tile reference.
    ///  @param[in]  ref    The tile reference of the tile to retrieve.
    /// @return The tile for the specified reference, or null if the
    ///    reference is invalid.
    function getTileByRef(ref: TdtTileRef): PdtMeshTile;

    /// The maximum number of tiles supported by the navigation mesh.
    /// @return The maximum number of tiles supported by the navigation mesh.
    function getMaxTiles(): Integer;

    /// Gets the tile at the specified index.
    ///  @param[in]  i    The tile index. [Limit: 0 >= index < #getMaxTiles()]
    /// @return The tile at the specified index.
    function getTile(i: Integer): PdtMeshTile;

    /// Gets the tile and polygon for the specified polygon reference.
    ///  @param[in]    ref    The reference for the a polygon.
    ///  @param[out]  tile  The tile containing the polygon.
    ///  @param[out]  poly  The polygon.
    /// @return The status flags for the operation.
    function getTileAndPolyByRef(ref: TdtPolyRef; tile: PPdtMeshTile; poly: PPdtPoly): TdtStatus;

    /// Returns the tile and polygon for the specified polygon reference.
    ///  @param[in]    ref    A known valid reference for a polygon.
    ///  @param[out]  tile  The tile containing the polygon.
    ///  @param[out]  poly  The polygon.
    procedure getTileAndPolyByRefUnsafe(ref: TdtPolyRef; tile: PPdtMeshTile; poly: PPdtPoly);

    /// Checks the validity of a polygon reference.
    ///  @param[in]  ref    The polygon reference to check.
    /// @return True if polygon reference is valid for the navigation mesh.
    function isValidPolyRef(ref: TdtPolyRef): Boolean;

    /// Gets the polygon reference for the tile's base polygon.
    ///  @param[in]  tile    The tile.
    /// @return The polygon reference for the base polygon in the specified tile.
    function getPolyRefBase(tile: PdtMeshTile): TdtPolyRef;

    /// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
    ///  @param[in]    prevRef    The reference of the polygon before the connection.
    ///  @param[in]    polyRef    The reference of the off-mesh connection polygon.
    ///  @param[out]  startPos  The start position of the off-mesh connection. [(x, y, z)]
    ///  @param[out]  endPos    The end position of the off-mesh connection. [(x, y, z)]
    /// @return The status flags for the operation.
    function getOffMeshConnectionPolyEndPoints(prevRef, polyRef: TdtPolyRef; startPos, endPos: PSingle): TdtStatus;

    /// Gets the specified off-mesh connection.
    ///  @param[in]  ref    The polygon reference of the off-mesh connection.
    /// @return The specified off-mesh connection, or null if the polygon reference is not valid.
    function getOffMeshConnectionByRef(ref: TdtPolyRef): PdtOffMeshConnection;

    /// @}

    /// @{
    /// @name State Management
    /// These functions do not effect #dtTileRef or #dtPolyRef's.

    /// Sets the user defined flags for the specified polygon.
    ///  @param[in]  ref    The polygon reference.
    ///  @param[in]  flags  The new flags for the polygon.
    /// @return The status flags for the operation.
    function setPolyFlags(ref: TdtPolyRef; flags: Word): TdtStatus;

    /// Gets the user defined flags for the specified polygon.
    ///  @param[in]    ref        The polygon reference.
    ///  @param[out]  resultFlags    The polygon flags.
    /// @return The status flags for the operation.
    function getPolyFlags(ref: TdtPolyRef; resultFlags: PWord): TdtStatus;

    /// Sets the user defined area for the specified polygon.
    ///  @param[in]  ref    The polygon reference.
    ///  @param[in]  area  The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
    /// @return The status flags for the operation.
    function setPolyArea(ref: TdtPolyRef; area: Byte): TdtStatus;

    /// Gets the user defined area for the specified polygon.
    ///  @param[in]    ref      The polygon reference.
    ///  @param[out]  resultArea  The area id for the polygon.
    /// @return The status flags for the operation.
    function getPolyArea(ref: TdtPolyRef; resultArea: PByte): TdtStatus;

    /// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
    ///  @param[in]  tile  The tile.
    /// @return The size of the buffer required to store the state.
    function getTileStateSize(tile: PdtMeshTile): Integer;

    /// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
    ///  @param[in]    tile      The tile.
    ///  @param[out]  data      The buffer to store the tile's state in.
    ///  @param[in]    maxDataSize    The size of the data buffer. [Limit: >= #getTileStateSize]
    /// @return The status flags for the operation.
    function storeTileState(tile: PdtMeshTile; data: PByte; maxDataSize: Integer): TdtStatus;

    /// Restores the state of the tile.
    ///  @param[in]  tile      The tile.
    ///  @param[in]  data      The new state. (Obtained from #storeTileState.)
    ///  @param[in]  maxDataSize    The size of the state within the data buffer.
    /// @return The status flags for the operation.
    function restoreTileState(tile: PdtMeshTile; data: PByte; maxDataSize: Integer): TdtStatus;

    /// @}

    /// @{
    /// @name Encoding and Decoding
    /// These functions are generally meant for internal use only.

    /// Derives a standard polygon reference.
    ///  @note This function is generally meant for internal use only.
    ///  @param[in]  salt  The tile's salt value.
    ///  @param[in]  it    The index of the tile.
    ///  @param[in]  ip    The index of the polygon within the tile.
    function encodePolyId(salt, it, ip: Cardinal): TdtPolyRef;

    /// Decodes a standard polygon reference.
    ///  @note This function is generally meant for internal use only.
    ///  @param[in]  ref   The polygon reference to decode.
    ///  @param[out]  salt  The tile's salt value.
    ///  @param[out]  it    The index of the tile.
    ///  @param[out]  ip    The index of the polygon within the tile.
    ///  @see #encodePolyId
    procedure decodePolyId(ref: TdtPolyRef; salt, it, ip: PCardinal);

    /// Extracts a tile's salt value from the specified polygon reference.
    ///  @note This function is generally meant for internal use only.
    ///  @param[in]  ref    The polygon reference.
    ///  @see #encodePolyId
    function decodePolyIdSalt(ref: TdtPolyRef): Cardinal;

    /// Extracts the tile's index from the specified polygon reference.
    ///  @note This function is generally meant for internal use only.
    ///  @param[in]  ref    The polygon reference.
    ///  @see #encodePolyId
    function decodePolyIdTile(ref: TdtPolyRef): Cardinal;

    /// Extracts the polygon's index (within its tile) from the specified polygon reference.
    ///  @note This function is generally meant for internal use only.
    ///  @param[in]  ref    The polygon reference.
    ///  @see #encodePolyId
    function decodePolyIdPoly(ref: TdtPolyRef): Cardinal;

    /// @end;

    procedure SaveToStream(aStream: TMemoryStream);
  private
    m_params: TdtNavMeshParams;      ///< Current initialization params. TODO: do not store this info twice.
    m_orig: array [0..2] of Single;          ///< Origin of the tile (0,0)
    m_tileWidth, m_tileHeight: Single;  ///< Dimensions of each tile.
    m_maxTiles: Integer;            ///< Max number of tiles.
    m_tileLutSize: Integer;          ///< Tile hash lookup size (must be pot).
    m_tileLutMask: Integer;          ///< Tile hash lookup mask.

    m_posLookup: PPointer;      ///< Tile hash lookup.
    m_nextFree: PdtMeshTile;        ///< Freelist of tiles.
    m_tiles: PdtMeshTile;        ///< List of tiles.

  {$ifndef DT_POLYREF64}
    m_saltBits: Cardinal;      ///< Number of salt bits in the tile ID.
    m_tileBits: Cardinal;      ///< Number of tile bits in the tile ID.
    m_polyBits: Cardinal;      ///< Number of poly bits in the tile ID.
  {$endif}

    /// Returns pointer to tile in the tile array.
    //function getTile(i: Integer): PdtMeshTile;

    /// Returns neighbour tile based on side.
    //function getTilesAt(x, y: Integer; tiles: Pointer; maxTiles: Integer): Integer;

    /// Returns neighbour tile based on side.
    function getNeighbourTilesAt(x, y, side: Integer; tiles: PPdtMeshTile; maxTiles: Integer): Integer;

    /// Returns all polygons in neighbour tile based on portal defined by the segment.
    function findConnectingPolys(va, vb: PSingle;
      tile: PdtMeshTile; side: Integer;
      con: PdtPolyRef; conarea: PSingle; maxcon: Integer): Integer;

    /// Builds internal polygons links for a tile.
    procedure connectIntLinks(tile: PdtMeshTile);
    /// Builds internal polygons links for a tile.
    procedure baseOffMeshLinks(tile: PdtMeshTile);

    /// Builds external polygon links for a tile.
    procedure connectExtLinks(tile, target: PdtMeshTile; side: Integer);
    /// Builds external polygon links for a tile.
    procedure connectExtOffMeshLinks(tile, target: PdtMeshTile; side: Integer);

    /// Removes external links at specified side.
    procedure unconnectExtLinks(tile, target: PdtMeshTile);


    // TODO: These methods are duplicates from dtNavMeshQuery, but are needed for off-mesh connection finding.

    /// Queries polygons within a tile.
    function queryPolygonsInTile(tile: PdtMeshTile; qmin, qmax: PSingle; polys: PdtPolyRef; maxPolys: Integer): Integer;
    /// Find nearest polygon within a tile.
    function findNearestPolyInTile(tile: PdtMeshTile; center, extents, nearestPt: PSingle): TdtPolyRef;
    /// Returns closest point on polygon.
    procedure closestPointOnPoly(ref: TdtPolyRef; pos, closest: PSingle; posOverPoly: PBoolean);
  end;

  /// Allocates a navigation mesh object using the Detour allocator.
  /// @return A navigation mesh that is ready for initialization, or null on failure.
  ///  @ingroup detour
  //dtNavMesh* dtAllocNavMesh();

  /// Frees the specified navigation mesh object using the Detour allocator.
  ///  @param[in]  navmesh    A navigation mesh allocated using #dtAllocNavMesh
  ///  @ingroup detour
  //void dtFreeNavMesh(dtNavMesh* navmesh);

implementation


  /// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
  procedure TdtPoly.setArea(a: Byte); begin areaAndtype := (areaAndtype and $c0) or (a and $3f); end;

  /// Sets the polygon type. (See: #dtPolyTypes.)
  procedure TdtPoly.setType(t: Byte); begin areaAndtype := (areaAndtype and $3f) or (t shl 6); end;

  /// Gets the user defined area id.
  function TdtPoly.getArea(): Byte; begin Result := areaAndtype and $3f; end;

  /// Gets the polygon type. (See: #dtPolyTypes)
  function TdtPoly.getType(): Byte; begin Result := areaAndtype shr 6; end;


///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

(**

@typedef dtPolyRef
@par

Polygon references are subject to the same invalidate/preserve/restore
rules that apply to #dtTileRef's.  If the #dtTileRef for the polygon's
tile changes, the polygon reference becomes invalid.

Changing a polygon's flags, area id, etc. does not impact its polygon
reference.

@typedef dtTileRef
@par

The following changes will invalidate a tile reference:

- The referenced tile has been removed from the navigation mesh.
- The navigation mesh has been initialized using a different set
  of #dtNavMeshParams.

A tile reference is preserved/restored if the tile is added to a navigation
mesh initialized with the original #dtNavMeshParams and is added at the
original reference location. (E.g. The lastRef parameter is used with
dtNavMesh::addTile.)

Basically, if the storage structure of a tile changes, its associated
tile reference changes.


@var unsigned short dtPoly::neis[DT_VERTS_PER_POLYGON]
@par

Each entry represents data for the edge starting at the vertex of the same index.
E.g. The entry at index n represents the edge data for vertex[n] to vertex[n+1].

A value of zero indicates the edge has no polygon connection. (It makes up the
border of the navigation mesh.)

The information can be extracted as follows:
@code
neighborRef = neis[n] & 0xff; // Get the neighbor polygon reference.

if (neis[n] & #DT_EX_LINK)
begin
    // The edge is an external (portal) edge.
end;
@endcode

@var float dtMeshHeader::bvQuantFactor
@par

This value is used for converting between world and bounding volume coordinates.
For example:
@code
const float cs = 1.0f / tile.header.bvQuantFactor;
const dtBVNode* n = &tile.bvTree[i];
if (n.i >= 0)
begin
    // This is a leaf node.
    float worldMinX = tile.header.bmin[0] + n.bmin[0]*cs;
    float worldMinY = tile.header.bmin[0] + n.bmin[1]*cs;
    // Etc...
end;
@endcode

@struct dtMeshTile
@par

Tiles generally only exist within the context of a dtNavMesh object.

Some tile content is optional.  For example, a tile may not contain any
off-mesh connections.  In this case the associated pointer will be null.

If a detail mesh exists it will share vertices with the base polygon mesh.
Only the vertices unique to the detail mesh will be stored in #detailVerts.

@warning Tiles returned by a dtNavMesh object are not guarenteed to be populated.
For example: The tile at a location might not have been loaded yet, or may have been removed.
In this case, pointers will be null.  So if in doubt, check the polygon count in the
tile's header to determine if a tile has polygons defined.

@var float dtOffMeshConnection::pos[6]
@par

For a properly built navigation mesh, vertex A will always be within the bounds of the mesh.
Vertex B is not required to be within the bounds of the mesh.

*)


function overlapSlabs(amin, amax, bmin, bmax: PSingle; px, py: Single): Boolean;
var minx,maxx,ad,ak,bd,bk,aminy,amaxy,bminy,bmaxy,dmin,dmax,thr: Single;
begin
  // Check for horizontal overlap.
  // The segment is shrunken a little so that slabs which touch
  // at end points are not connected.
  minx := dtMax(amin[0]+px,bmin[0]+px);
  maxx := dtMin(amax[0]-px,bmax[0]-px);
  if (minx > maxx) then
    Exit(false);

  // Check vertical overlap.
  ad := (amax[1]-amin[1]) / (amax[0]-amin[0]);
  ak := amin[1] - ad*amin[0];
  bd := (bmax[1]-bmin[1]) / (bmax[0]-bmin[0]);
  bk := bmin[1] - bd*bmin[0];
  aminy := ad*minx + ak;
  amaxy := ad*maxx + ak;
  bminy := bd*minx + bk;
  bmaxy := bd*maxx + bk;
  dmin := bminy - aminy;
  dmax := bmaxy - amaxy;

  // Crossing segments always overlap.
  if (dmin*dmax < 0) then
    Exit(true);

  // Check for overlap at endpoints.
  thr := Sqr(py*2);
  if (dmin*dmin <= thr) or (dmax*dmax <= thr) then
    Exit(true);

  Result := false;
end;

function getSlabCoord(va: PSingle; side: Integer): Single;
begin
  if (side = 0) or (side = 4) then
    Exit(va[0])
  else if (side = 2) or (side = 6) then
    Exit(va[2]);
  Result := 0;
end;

procedure calcSlabEndPoints(va, vb, bmin,bmax: PSingle; side: Integer);
begin
  if (side = 0) or (side = 4) then
  begin
    if (va[2] < vb[2]) then
    begin
      bmin[0] := va[2];
      bmin[1] := va[1];
      bmax[0] := vb[2];
      bmax[1] := vb[1];
    end
    else
    begin
      bmin[0] := vb[2];
      bmin[1] := vb[1];
      bmax[0] := va[2];
      bmax[1] := va[1];
    end;
  end
  else if (side = 2) or (side = 6) then
  begin
    if (va[0] < vb[0]) then
    begin
      bmin[0] := va[0];
      bmin[1] := va[1];
      bmax[0] := vb[0];
      bmax[1] := vb[1];
    end
    else
    begin
      bmin[0] := vb[0];
      bmin[1] := vb[1];
      bmax[0] := va[0];
      bmax[1] := va[1];
    end;
  end;
end;

function computeTileHash(x, y, mask: Integer): Integer;
const h1: Cardinal = $8da6b343; // Large multiplicative constants;
const h2: Cardinal = $d8163841; // here arbitrarily chosen primes
var n: Int64;
begin
{$Q-}
  n := h1 * x + h2 * y;
  Result := Integer(n and mask);
{$Q+}
end;

function allocLink(tile: PdtMeshTile): Cardinal;
var link: Cardinal;
begin
  if (tile.linksFreeList = DT_NULL_LINK) then
    Exit(DT_NULL_LINK);
  link := tile.linksFreeList;
  tile.linksFreeList := tile.links[link].next;
  Result := link;
end;

procedure freeLink(tile: PdtMeshTile; link: Cardinal);
begin
  tile.links[link].next := tile.linksFreeList;
  tile.linksFreeList := link;
end;


{function dtAllocNavMesh(): PdtNavMesh;
begin
  void* mem := dtAlloc(sizeof(dtNavMesh), DT_ALLOC_PERM);
  if (!mem) return 0;
  return new(mem) dtNavMesh;
end;

/// @par
///
/// This function will only free the memory for tiles with the #DT_TILE_FREE_DATA
/// flag set.
procedure dtFreeNavMesh(dtNavMesh* navmesh);
begin
  if (!navmesh) return;
  navmesh.~dtNavMesh();
  dtFree(navmesh);
end;}

//////////////////////////////////////////////////////////////////////////////////////////

(**
@class dtNavMesh

The navigation mesh consists of one or more tiles defining three primary types of structural data:

A polygon mesh which defines most of the navigation graph. (See rcPolyMesh for its structure.)
A detail mesh used for determining surface height on the polygon mesh. (See rcPolyMeshDetail for its structure.)
Off-mesh connections, which define custom point-to-point edges within the navigation graph.

The general build process is as follows:

-# Create rcPolyMesh and rcPolyMeshDetail data using the Recast build pipeline.
-# Optionally, create off-mesh connection data.
-# Combine the source data into a dtNavMeshCreateParams structure.
-# Create a tile data array using dtCreateNavMeshData().
-# Allocate at dtNavMesh object and initialize it. (For single tile navigation meshes,
   the tile data is loaded during this step.)
-# For multi-tile navigation meshes, load the tile data using dtNavMesh::addTile().

Notes:

- This class is usually used in conjunction with the dtNavMeshQuery class for pathfinding.
- Technically, all navigation meshes are tiled. A 'solo' mesh is simply a navigation mesh initialized
  to have only a single tile.
- This class does not implement any asynchronous methods. So the ::dtStatus result of all methods will
  always contain either a success or failure flag.

@see dtNavMeshQuery, dtCreateNavMeshData, dtNavMeshCreateParams, #dtAllocNavMesh, #dtFreeNavMesh
*)

constructor TdtNavMesh.Create();
begin
  FillChar(m_params, sizeof(TdtNavMeshParams), 0);
end;

destructor TdtNavMesh.Destroy;
var i: Integer;
begin
  for i := 0 to m_maxTiles - 1 do
  begin
    if (m_tiles[i].flags and DT_TILE_FREE_DATA) <> 0 then
    begin
      FreeMem(m_tiles[i].data);
      m_tiles[i].data := nil;
      m_tiles[i].dataSize := 0;
    end;
  end;
  FreeMem(m_posLookup);
  FreeMem(m_tiles);
  inherited;
end;

function TdtNavMesh.init(params: PdtNavMeshParams): TdtStatus;
var i: Integer;
begin
  Move(params^, m_params, sizeof(TdtNavMeshParams));
  dtVcopy(@m_orig[0], @params.orig[0]);
  m_tileWidth := params.tileWidth;
  m_tileHeight := params.tileHeight;

  // Init tiles
  m_maxTiles := params.maxTiles;
  m_tileLutSize := dtNextPow2(params.maxTiles div 4);
  if (m_tileLutSize = 0) then m_tileLutSize := 1;
  m_tileLutMask := m_tileLutSize-1;

  GetMem(m_tiles, sizeof(TdtMeshTile)*m_maxTiles);
  GetMem(m_posLookup, sizeof(PdtMeshTile)*m_tileLutSize);

  FillChar(m_tiles[0], sizeof(TdtMeshTile)*m_maxTiles, 0);
  FillChar(m_posLookup[0], sizeof(PdtMeshTile)*m_tileLutSize, 0);
  m_nextFree := nil;
  for i := m_maxTiles-1 downto 0 do
  begin
    m_tiles[i].salt := 1;
    m_tiles[i].next := m_nextFree;
    m_nextFree := @m_tiles[i];
  end;

  // Init ID generator values.
{$ifndef DT_POLYREF64}
  m_tileBits := dtIlog2(dtNextPow2(Cardinal(params.maxTiles)));
  m_polyBits := dtIlog2(dtNextPow2(Cardinal(params.maxPolys)));
  // Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
  m_saltBits := dtMin(Cardinal(31), 32 - m_tileBits - m_polyBits);

  if (m_saltBits < 10) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
{$endif}

  Result := DT_SUCCESS;
end;

function TdtNavMesh.init(data: PByte; dataSize, flags: Integer): TdtStatus;
var header: PdtMeshHeader; params: TdtNavMeshParams; status: TdtStatus;
begin
  // Make sure the data is in right format.
  header := PdtMeshHeader(data);
  if (header.magic <> DT_NAVMESH_MAGIC) then
    Exit(DT_FAILURE or DT_WRONG_MAGIC);
  if (header.version <> DT_NAVMESH_VERSION) then
    Exit(DT_FAILURE or DT_WRONG_VERSION);

  dtVcopy(@params.orig[0], @header.bmin[0]);
  params.tileWidth := header.bmax[0] - header.bmin[0];
  params.tileHeight := header.bmax[2] - header.bmin[2];
  params.maxTiles := 1;
  params.maxPolys := header.polyCount;

  status := init(@params);
  if (dtStatusFailed(status)) then
    Exit(status);

  Result := addTile(data, dataSize, flags, 0, nil);
end;

/// @par
///
/// @note The parameters are created automatically when the single tile
/// initialization is performed.
function TdtNavMesh.getParams(): PdtNavMeshParams;
begin
  Result := @m_params;
end;

//////////////////////////////////////////////////////////////////////////////////////////
function TdtNavMesh.findConnectingPolys(va, vb: PSingle;
  tile: PdtMeshTile; side: Integer;
  con: PdtPolyRef; conarea: PSingle; maxcon: Integer): Integer;
var amin, amax,bmin,bmax: array [0..2] of Single; apos: Single; m: Word; i,j,n,nv: Integer; base: TdtPolyRef; poly: PdtPoly;
vc,vd: PSingle; bpos: Single;
begin
  if (tile = nil) then Exit(0);

  calcSlabEndPoints(va, vb, @amin[0], @amax[0], side);
  apos := getSlabCoord(va, side);

  // Remove links pointing to 'side' and compact the links array.
  m := DT_EXT_LINK or Word(side);
  n := 0;

  base := getPolyRefBase(tile);

  for i := 0 to tile.header.polyCount - 1 do
  begin
    poly := @tile.polys[i];
    nv := poly.vertCount;
    for j := 0 to nv - 1 do
    begin
      // Skip edges which do not point to the right side.
      if (poly.neis[j] <> m) then continue;

      vc := @tile.verts[poly.verts[j]*3];
      vd := @tile.verts[poly.verts[(j+1) mod nv]*3];
      bpos := getSlabCoord(vc, side);

      // Segments are not close enough.
      if (Abs(apos-bpos) > 0.01) then
        continue;

      // Check if the segments touch.
      calcSlabEndPoints(vc,vd, @bmin[0],@bmax[0], side);

      if (not overlapSlabs(@amin[0],@amax[0], @bmin[0],@bmax[0], 0.01, tile.header.walkableClimb)) then continue;

      // Add return value.
      if (n < maxcon) then
      begin
        conarea[n*2+0] := dtMax(amin[0], bmin[0]);
        conarea[n*2+1] := dtMin(amax[0], bmax[0]);
        con[n] := base or TdtPolyRef(i);
        Inc(n);
      end;
      break;
    end;
  end;
  Result := n;
end;

procedure TdtNavMesh.unconnectExtLinks(tile, target: PdtMeshTile);
var targetNum: Cardinal; i: Integer; poly: PdtPoly; j,pj,nj: Cardinal;
begin
  if (tile = nil) or (target = nil) then Exit;

  targetNum := decodePolyIdTile(getTileRef(target));

  for i := 0 to tile.header.polyCount - 1 do
  begin
    poly := @tile.polys[i];
    j := poly.firstLink;
    pj := DT_NULL_LINK;
    while (j <> DT_NULL_LINK) do
    begin
      if (tile.links[j].side <> $ff) and
        (decodePolyIdTile(tile.links[j].ref) = targetNum) then
      begin
        // Revove link.
        nj := tile.links[j].next;
        if (pj = DT_NULL_LINK) then
          poly.firstLink := nj
        else
          tile.links[pj].next := nj;
        freeLink(tile, j);
        j := nj;
      end
      else
      begin
        // Advance
        pj := j;
        j := tile.links[j].next;
      end;
    end;
  end;
end;

procedure TdtNavMesh.connectExtLinks(tile, target: PdtMeshTile; side: Integer);
var i,j,k: Integer; poly: PdtPoly; nv,dir: Integer; va,vb: PSingle; nei: array [0..3] of TdtPolyRef; neia: array [0..7] of Single;
nnei: Integer; idx: Cardinal; link: PdtLink; tmin,tmax: Single;
begin
  if (tile = nil) then Exit;

  // Connect border links.
  for i := 0 to tile.header.polyCount - 1 do
  begin
    poly := @tile.polys[i];

    // Create new links.
//    unsigned short m := DT_EXT_LINK | (unsigned short)side;

    nv := poly.vertCount;
    for j := 0 to nv - 1 do
    begin
      // Skip non-portal edges.
      if ((poly.neis[j] and DT_EXT_LINK) = 0) then
        continue;

      dir := Integer(poly.neis[j] and $ff);
      if (side <> -1) and (dir <> side) then
        continue;

      // Create new links
      va := @tile.verts[poly.verts[j]*3];
      vb := @tile.verts[poly.verts[(j+1) mod nv]*3];
      nnei := findConnectingPolys(va,vb, target, dtOppositeTile(dir), @nei[0],@neia[0],4);
      for k := 0 to nnei - 1 do
      begin
        idx := allocLink(tile);
        if (idx <> DT_NULL_LINK) then
        begin
          link := @tile.links[idx];
          link.ref := nei[k];
          link.edge := Byte(j);
          link.side := Byte(dir);

          link.next := poly.firstLink;
          poly.firstLink := idx;

          // Compress portal limits to a byte value.
          if (dir = 0) or (dir = 4) then
          begin
            tmin := (neia[k*2+0]-va[2]) / (vb[2]-va[2]);
            tmax := (neia[k*2+1]-va[2]) / (vb[2]-va[2]);
            if (tmin > tmax) then
              dtSwap(tmin,tmax);
            link.bmin := Byte(Trunc(dtClamp(tmin, 0.0, 1.0)*255.0));
            link.bmax := Byte(Trunc(dtClamp(tmax, 0.0, 1.0)*255.0));
          end
          else if (dir = 2) or (dir = 6) then
          begin
            tmin := (neia[k*2+0]-va[0]) / (vb[0]-va[0]);
            tmax := (neia[k*2+1]-va[0]) / (vb[0]-va[0]);
            if (tmin > tmax) then
              dtSwap(tmin,tmax);
            link.bmin := Byte(Trunc(dtClamp(tmin, 0.0, 1.0)*255.0));
            link.bmax := Byte(Trunc(dtClamp(tmax, 0.0, 1.0)*255.0));
          end;
        end;
      end;
    end;
  end;
end;

procedure TdtNavMesh.connectExtOffMeshLinks(tile, target: PdtMeshTile; side: Integer);
var oppositeSide: Byte; i: Integer; targetCon: PdtOffMeshConnection; targetPoly: PdtPoly; ext,nearestPt: array [0..2] of Single;
p,v: PSingle; ref: TdtPolyRef; idx,tidx: Cardinal; link: PdtLink; landPolyIdx: Word; landPoly: PdtPoly;
begin
  if (tile = nil) then Exit;

  // Connect off-mesh links.
  // We are interested on links which land from target tile to this tile.
  if (side = -1) then oppositeSide := $ff else oppositeSide := Byte(dtOppositeTile(side));

  for i := 0 to target.header.offMeshConCount - 1 do
  begin
    targetCon := @target.offMeshCons[i];
    if (targetCon.side <> oppositeSide) then
      continue;

    targetPoly := @target.polys[targetCon.poly];
    // Skip off-mesh connections which start location could not be connected at all.
    if (targetPoly.firstLink = DT_NULL_LINK) then
      continue;

    ext[0] := targetCon.rad; ext[1] := target.header.walkableClimb; ext[2] := targetCon.rad;

    // Find polygon to connect to.
    p := @targetCon.pos[3];

    ref := findNearestPolyInTile(tile, p, @ext[0], @nearestPt[0]);
    if (ref = 0) then
      continue;
    // findNearestPoly may return too optimistic results, further check to make sure.
    if (Sqr(nearestPt[0]-p[0])+Sqr(nearestPt[2]-p[2]) > Sqr(targetCon.rad)) then
      continue;
    // Make sure the location is on current mesh.
    v := @target.verts[targetPoly.verts[1]*3];
    dtVcopy(v, @nearestPt[0]);

    // Link off-mesh connection to target poly.
    idx := allocLink(target);
    if (idx <> DT_NULL_LINK) then
    begin
      link := @target.links[idx];
      link.ref := ref;
      link.edge := Byte(1);
      link.side := oppositeSide;
      link.bmin := 0; link.bmax := 0;
      // Add to linked list.
      link.next := targetPoly.firstLink;
      targetPoly.firstLink := idx;
    end;

    // Link target poly to off-mesh connection.
    if (targetCon.flags and DT_OFFMESH_CON_BIDIR <> 0) then
    begin
      tidx := allocLink(tile);
      if (tidx <> DT_NULL_LINK) then
      begin
        landPolyIdx := Word(decodePolyIdPoly(ref));
        landPoly := @tile.polys[landPolyIdx];
        link := @tile.links[tidx];
        link.ref := getPolyRefBase(target) or TdtPolyRef(targetCon.poly);
        link.edge := $ff;
        link.side := Byte(IfThen(side = -1, $ff, side));
        link.bmin := 0; link.bmax := 0;
        // Add to linked list.
        link.next := landPoly.firstLink;
        landPoly.firstLink := tidx;
      end;
    end;
  end;

end;

procedure TdtNavMesh.connectIntLinks(tile: PdtMeshTile);
var base: TdtPolyRef; i,j: Integer; poly: PdtPoly; idx: Cardinal; link: PdtLink;
begin
  if (tile = nil) then Exit;

  base := getPolyRefBase(tile);

  for i := 0 to tile.header.polyCount - 1 do
  begin
    poly := @tile.polys[i];
    poly.firstLink := DT_NULL_LINK;

    if (poly.getType() = DT_POLYTYPE_OFFMESH_CONNECTION) then
      continue;

    // Build edge links backwards so that the links will be
    // in the linked list from lowest index to highest.
    for j := poly.vertCount-1 downto 0 do
    begin
      // Skip hard and non-internal edges.
      if (poly.neis[j] = 0) or ((poly.neis[j] and DT_EXT_LINK) <> 0) then continue;

      idx := allocLink(tile);
      if (idx <> DT_NULL_LINK) then
      begin
        link := @tile.links[idx];
        link.ref := base or TdtPolyRef(poly.neis[j]-1);
        link.edge := Byte(j);
        link.side := $ff;
        link.bmin := 0; link.bmax := 0;
        // Add to linked list.
        link.next := poly.firstLink;
        poly.firstLink := idx;
      end;
    end;
  end;
end;

procedure TdtNavMesh.baseOffMeshLinks(tile: PdtMeshTile);
var base: TdtPolyRef; i: Integer; con: PdtOffMeshConnection; poly: PdtPoly; ext,nearestPt: array [0..2] of Single;
p,v: PSingle; ref: TdtPolyRef; idx,tidx: Cardinal; link: PdtLink; landPolyIdx: Word; landPoly: PdtPoly;
begin
  if (tile = nil) then Exit;

  base := getPolyRefBase(tile);

  // Base off-mesh connection start points.
  for i := 0 to tile.header.offMeshConCount - 1 do
  begin
    con := @tile.offMeshCons[i];
    poly := @tile.polys[con.poly];

    ext[0] := con.rad; ext[1] := tile.header.walkableClimb; ext[2] := con.rad;

    // Find polygon to connect to.
    p := @con.pos[0]; // First vertex

    ref := findNearestPolyInTile(tile, p, @ext[0], @nearestPt[0]);
    if (ref = 0) then continue;
    // findNearestPoly may return too optimistic results, further check to make sure.
    if (Sqr(nearestPt[0]-p[0])+Sqr(nearestPt[2]-p[2]) > Sqr(con.rad)) then
      continue;
    // Make sure the location is on current mesh.
    v := @tile.verts[poly.verts[0]*3];
    dtVcopy(v, @nearestPt[0]);

    // Link off-mesh connection to target poly.
    idx := allocLink(tile);
    if (idx <> DT_NULL_LINK) then
    begin
      link := @tile.links[idx];
      link.ref := ref;
      link.edge := 0;
      link.side := $ff;
      link.bmin := 0; link.bmax := 0;
      // Add to linked list.
      link.next := poly.firstLink;
      poly.firstLink := idx;
    end;

    // Start end-point is always connect back to off-mesh connection.
    tidx := allocLink(tile);
    if (tidx <> DT_NULL_LINK) then
    begin
      landPolyIdx := Word(decodePolyIdPoly(ref));
      landPoly := @tile.polys[landPolyIdx];
      link := @tile.links[tidx];
      link.ref := base or TdtPolyRef(con.poly);
      link.edge := $ff;
      link.side := $ff;
      link.bmin := 0; link.bmax := 0;
      // Add to linked list.
      link.next := landPoly.firstLink;
      landPoly.firstLink := tidx;
    end;
  end;
end;

procedure TdtNavMesh.closestPointOnPoly(ref: TdtPolyRef; pos, closest: PSingle; posOverPoly: PBoolean);
var tile: PdtMeshTile; poly: PdtPoly; v0,v1,va,vb: PSingle; d0,d1,u: Single; ip: Cardinal; pd: PdtPolyDetail;
verts: array [0..DT_VERTS_PER_POLYGON*3-1] of Single; edged,edget: array [0..DT_VERTS_PER_POLYGON-1] of Single;
nv,i,imin,j,k: Integer; dmin,h: Single; t: PByte; v: array [0..2] of PSingle;
begin
  tile := nil;
  poly := nil;
  getTileAndPolyByRefUnsafe(ref, @tile, @poly);

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
    Exit;
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
end;

function TdtNavMesh.findNearestPolyInTile(tile: PdtMeshTile; center, extents, nearestPt: PSingle): TdtPolyRef;
var bmin,bmax,closestPtPoly,diff: array [0..2] of Single; polys: array[0..127] of TdtPolyRef; polyCount,i: Integer; nearest: TdtPolyRef;
nearestDistanceSqr: Single; ref: TdtPolyRef; posOverPoly: Boolean; d: Single;
begin
  dtVsub(@bmin[0], center, extents);
  dtVadd(@bmax[0], center, extents);

  // Get nearby polygons from proximity grid.
  polyCount := queryPolygonsInTile(tile, @bmin[0], @bmax[0], @polys[0], 128);

  // Find nearest polygon amongst the nearby polygons.
  nearest := 0;
  nearestDistanceSqr := MaxSingle;
  for i := 0 to polyCount - 1 do
  begin
    ref := polys[i];
    posOverPoly := false;
    closestPointOnPoly(ref, center, @closestPtPoly[0], @posOverPoly);

    // If a point is directly over a polygon and closer than
    // climb height, favor that instead of straight line nearest point.
    dtVsub(@diff[0], center, @closestPtPoly[0]);
    if (posOverPoly) then
    begin
      d := Abs(diff[1]) - tile.header.walkableClimb;
      d := IfThen(d > 0, d*d, 0);
    end
    else
    begin
      d := dtVlenSqr(@diff[0]);
    end;

    if (d < nearestDistanceSqr) then
    begin
      dtVcopy(nearestPt, @closestPtPoly[0]);
      nearestDistanceSqr := d;
      nearest := ref;
    end;
  end;

  Result := nearest;
end;

function TdtNavMesh.queryPolygonsInTile(tile: PdtMeshTile; qmin, qmax: PSingle; polys: PdtPolyRef; maxPolys: Integer): Integer;
var node: PdtBVNode; &end: PdtBVNode; tbmin,tbmax: Psingle; qfac: Single; bmin,bmax: array [0..2] of Word; bminf,bmaxf: array [0..2] of Single;
minx,miny,minz,maxx,maxy,maxz: Single; base: TdtPolyRef; i,j,n,escapeIndex: Integer; overlap,isLeafNode: Boolean; p: PdtPoly;
v: PSingle;
begin
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
    base := getPolyRefBase(tile);
    n := 0;
    while (node < &end) do
    begin
      overlap := dtOverlapQuantBounds(@bmin[0], @bmax[0], @node.bmin[0], @node.bmax[0]);
      isLeafNode := node.i >= 0;

      if (isLeafNode and overlap) then
      begin
        if (n < maxPolys) then
        begin
          polys[n] := base or TdtPolyRef(node.i);
          Inc(n);
        end;
      end;

      if (overlap) or (isLeafNode) then
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
    base := getPolyRefBase(tile);
    for i := 0 to tile.header.polyCount - 1 do
    begin
      p := @tile.polys[i];
      // Do not return off-mesh connection polygons.
      if (p.getType() = DT_POLYTYPE_OFFMESH_CONNECTION) then
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
          polys[n] := base or TdtPolyRef(i);
          Inc(n);
        end;
      end;
    end;
    Exit(n);
  end;
end;

/// @par
///
/// The add operation will fail if the data is in the wrong format, the allocated tile
/// space is full, or there is a tile already at the specified reference.
///
/// The lastRef parameter is used to restore a tile with the same tile
/// reference it had previously used.  In this case the #dtPolyRef's for the
/// tile will be restored to the same values they were before the tile was
/// removed.
///
/// @see dtCreateNavMeshData, #removeTile
function TdtNavMesh.addTile(data: PByte; dataSize, flags: Integer; lastRef: TdtTileRef; reslt: PdtTileRef): TdtStatus;
const MAX_NEIS = 32;
var header: PdtMeshHeader; tile: PdtMeshTile; tileIndex,h: Integer; target, prev: PdtMeshTile;
headerSize, vertsSize, polysSize, linksSize, detailMeshesSize, detailVertsSize, detailTrisSize, bvtreeSize, offMeshLinksSize: Integer;
d: PByte; i,j,nneis: Integer; neis: array [0..MAX_NEIS-1] of PdtMeshTile;
begin
  // Make sure the data is in right format.
  header := PdtMeshHeader(data);
  if (header.magic <> DT_NAVMESH_MAGIC) then
    Exit(DT_FAILURE or DT_WRONG_MAGIC);
  if (header.version <> DT_NAVMESH_VERSION) then
    Exit(DT_FAILURE or DT_WRONG_VERSION);

  // Make sure the location is free.
  if (getTileAt(header.x, header.y, header.layer) <> nil) then
    Exit(DT_FAILURE);

  // Allocate a tile.
  tile := nil;
  if (lastRef = 0) then
  begin
    if (m_nextFree <> nil) then
    begin
      tile := m_nextFree;
      m_nextFree := tile.next;
      tile.next := nil;
    end;
  end
  else
  begin
    // Try to relocate the tile to specific index with same salt.
    tileIndex := Integer(decodePolyIdTile(TdtPolyRef(lastRef)));
    if (tileIndex >= m_maxTiles) then
      Exit(DT_FAILURE or DT_OUT_OF_MEMORY);
    // Try to find the specific tile id from the free list.
    target := @m_tiles[tileIndex];
    prev := nil;
    tile := m_nextFree;
    while (tile <> nil) and (tile <> target) do
    begin
      prev := tile;
      tile := tile.next;
    end;
    // Could not find the correct location.
    if (tile <> target) then
      Exit(DT_FAILURE or DT_OUT_OF_MEMORY);
    // Remove from freelist
    if (prev = nil) then
      m_nextFree := tile.next
    else
      prev.next := tile.next;

    // Restore salt.
    tile.salt := decodePolyIdSalt(TdtPolyRef(lastRef));
  end;

  // Make sure we could allocate a tile.
  if (tile = nil) then
    Exit(DT_FAILURE or DT_OUT_OF_MEMORY);

  // Insert tile into the position lut.
  h := computeTileHash(header.x, header.y, m_tileLutMask);
  tile.next := PdtMeshTile(m_posLookup[h]);
  m_posLookup[h] := tile;

  // Patch header pointers.
  headerSize := dtAlign4(sizeof(TdtMeshHeader));
  vertsSize := dtAlign4(sizeof(Single)*3*header.vertCount);
  polysSize := dtAlign4(sizeof(TdtPoly)*header.polyCount);
  linksSize := dtAlign4(sizeof(TdtLink)*(header.maxLinkCount));
  detailMeshesSize := dtAlign4(sizeof(TdtPolyDetail)*header.detailMeshCount);
  detailVertsSize := dtAlign4(sizeof(Single)*3*header.detailVertCount);
  detailTrisSize := dtAlign4(sizeof(Byte)*4*header.detailTriCount);
  bvtreeSize := dtAlign4(sizeof(TdtBVNode)*header.bvNodeCount);
  offMeshLinksSize := dtAlign4(sizeof(TdtOffMeshConnection)*header.offMeshConCount);

  d := data + headerSize;
  tile.verts := PSingle(d); Inc(d, vertsSize);
  tile.polys := PdtPoly(d); Inc(d, polysSize);
  tile.links := PdtLink(d); Inc(d, linksSize);
  tile.detailMeshes := PdtPolyDetail(d); Inc(d, detailMeshesSize);
  tile.detailVerts := PSingle(d); Inc(d, detailVertsSize);
  tile.detailTris := PByte(d); Inc(d, detailTrisSize);
  tile.bvTree := PdtBVNode(d); Inc(d, bvtreeSize);
  tile.offMeshCons := PdtOffMeshConnection(d); Inc(d, offMeshLinksSize);

  // If there are no items in the bvtree, reset the tree pointer.
  if (bvtreeSize = 0) then
    tile.bvTree := nil;

  // Build links freelist
  tile.linksFreeList := 0;
  tile.links[header.maxLinkCount-1].next := DT_NULL_LINK;
  for i := 0 to header.maxLinkCount-1 - 1 do
    tile.links[i].next := i+1;

  // Init tile.
  tile.header := header;
  tile.data := data;
  tile.dataSize := dataSize;
  tile.flags := flags;

  connectIntLinks(tile);
  baseOffMeshLinks(tile);

  // Create connections with neighbour tiles.

  // Connect with layers in current tile.
  nneis := getTilesAt(header.x, header.y, @neis[0], MAX_NEIS);
  for j := 0 to nneis - 1 do
  begin
    if (neis[j] <> tile) then
    begin
      connectExtLinks(tile, neis[j], -1);
      connectExtLinks(neis[j], tile, -1);
    end;
    connectExtOffMeshLinks(tile, neis[j], -1);
    connectExtOffMeshLinks(neis[j], tile, -1);
  end;

  // Connect with neighbour tiles.
  for i := 0 to 7 do
  begin
    nneis := getNeighbourTilesAt(header.x, header.y, i, @neis[0], MAX_NEIS);
    for j := 0 to nneis - 1 do
    begin
      connectExtLinks(tile, neis[j], i);
      connectExtLinks(neis[j], tile, dtOppositeTile(i));
      connectExtOffMeshLinks(tile, neis[j], i);
      connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i));
    end;
  end;

  if (reslt <> nil) then
    reslt^ := getTileRef(tile);

  Result := DT_SUCCESS;
end;

function TdtNavMesh.getTileAt(x, y, layer: Integer): PdtMeshTile;
var h: Integer; tile: PdtMeshTile;
begin
  // Find tile based on hash.
  h := computeTileHash(x,y,m_tileLutMask);
  tile := m_posLookup[h];
  while (tile <> nil) do
  begin
    if (tile.header <> nil) and
      (tile.header.x = x) and
      (tile.header.y = y) and
      (tile.header.layer = layer) then
    begin
      Exit(tile);
    end;
    tile := tile.next;
  end;
  Result := nil;
end;

function TdtNavMesh.getNeighbourTilesAt(x, y, side: Integer; tiles: PPdtMeshTile; maxTiles: Integer): Integer;
var nx,ny: Integer;
begin
  nx := x; ny := y;
  case side of
    0: begin Inc(nx); end;
    1: begin Inc(nx); Inc(ny); end;
    2: begin Inc(ny); end;
    3: begin Dec(nx); Inc(ny); end;
    4: begin Dec(nx); end;
    5: begin Dec(nx); Dec(ny); end;
    6: begin Dec(ny); end;
    7: begin Inc(nx); Dec(ny); end;
  end;

  Result := getTilesAt(nx, ny, tiles, maxTiles);
end;

function TdtNavMesh.getTilesAt(x, y: Integer; tiles: PPdtMeshTile; maxTiles: Integer): Integer;
var n,h: Integer; tile: PdtMeshTile;
begin
  n := 0;

  // Find tile based on hash.
  h := computeTileHash(x,y,m_tileLutMask);
  tile := m_posLookup[h];
  while (tile <> nil) do
  begin
    if (tile.header <> nil) and
      (tile.header.x = x) and
      (tile.header.y = y) then
    begin
      if (n < maxTiles) then
      begin
        tiles[n] := tile;
        Inc(n);
      end;
    end;
    tile := tile.next;
  end;

  Result := n;
end;

function TdtNavMesh.getTileRefAt(x, y, layer: Integer): TdtTileRef;
var h: Integer; tile: PdtMeshTile;
begin
  // Find tile based on hash.
  h := computeTileHash(x,y,m_tileLutMask);
  tile := m_posLookup[h];
  while (tile <> nil) do
  begin
    if (tile.header <> nil) and
      (tile.header.x = x) and
      (tile.header.y = y) and
      (tile.header.layer = layer) then
    begin
      Exit(getTileRef(tile));
    end;
    tile := tile.next;
  end;
  Result := 0;
end;

function TdtNavMesh.getTileByRef(ref: TdtTileRef): PdtMeshTile;
var tileIndex, tileSalt: Cardinal; tile: PdtMeshTile;
begin
  if (ref = 0) then
    Exit(nil);
  tileIndex := decodePolyIdTile(TdtPolyRef(ref));
  tileSalt := decodePolyIdSalt(TdtPolyRef(ref));
  if (Integer(tileIndex) >= m_maxTiles) then
    Exit(nil);
  tile := @m_tiles[tileIndex];
  if (tile.salt <> tileSalt) then
    Exit(nil);
  Result := tile;
end;

function TdtNavMesh.getMaxTiles(): Integer;
begin
  Result := m_maxTiles;
end;

function TdtNavMesh.getTile(i: Integer): PdtMeshTile;
begin
  Result := @m_tiles[i];
end;

procedure TdtNavMesh.calcTileLoc(pos: PSingle; tx, ty: PInteger);
begin
  tx^ := floor((pos[0]-m_orig[0]) / m_tileWidth);
  ty^ := floor((pos[2]-m_orig[2]) / m_tileHeight);
end;

function TdtNavMesh.getTileAndPolyByRef(ref: TdtPolyRef; tile: PPdtMeshTile; poly: PPdtPoly): TdtStatus;
var salt, it, ip: Cardinal;
begin
  if (ref = 0) then Exit(DT_FAILURE);
  decodePolyId(ref, @salt, @it, @ip);
  if (it >= m_maxTiles) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (m_tiles[it].salt <> salt) or (m_tiles[it].header = nil) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (ip >= m_tiles[it].header.polyCount) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  tile^ := @m_tiles[it];
  poly^ := @m_tiles[it].polys[ip];
  Result := DT_SUCCESS;
end;

/// @par
///
/// @warning Only use this function if it is known that the provided polygon
/// reference is valid. This function is faster than #getTileAndPolyByRef, but
/// it does not validate the reference.
procedure TdtNavMesh.getTileAndPolyByRefUnsafe(ref: TdtPolyRef; tile: PPdtMeshTile; poly: PPdtPoly);
var salt, it, ip: Cardinal;
begin
  decodePolyId(ref, @salt, @it, @ip);
  tile^ := @m_tiles[it];
  poly^ := @m_tiles[it].polys[ip];
end;

function TdtNavMesh.isValidPolyRef(ref: TdtPolyRef): Boolean;
var salt, it, ip: Cardinal;
begin
  if (ref = 0) then Exit(false);
  decodePolyId(ref, @salt, @it, @ip);
  if (it >= m_maxTiles) then Exit(false);
  if (m_tiles[it].salt <> salt) or (m_tiles[it].header = nil) then Exit(false);
  if (ip >= m_tiles[it].header.polyCount) then Exit(false);
  Result := true;
end;

/// @par
///
/// This function returns the data for the tile so that, if desired,
/// it can be added back to the navigation mesh at a later point.
///
/// @see #addTile
function TdtNavMesh.removeTile(ref: TdtTileRef; data: PPointer; dataSize: PInteger): TdtStatus;
const MAX_NEIS = 32;
var tileIndex, tileSalt: Cardinal; tile,prev,cur: PdtMeshTile; i,j,h,nneis: Integer; neis: array [0..MAX_NEIS-1] of PdtMeshTile;
begin
  if (ref = 0) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  tileIndex := decodePolyIdTile(TdtPolyRef(ref));
  tileSalt := decodePolyIdSalt(TdtPolyRef(ref));
  if (tileIndex >= m_maxTiles) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);
  tile := @m_tiles[tileIndex];
  if (tile.salt <> tileSalt) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // Remove tile from hash lookup.
  h := computeTileHash(tile.header.x,tile.header.y,m_tileLutMask);
  prev := nil;
  cur := m_posLookup[h];
  while (cur <> nil) do
  begin
    if (cur = tile) then
    begin
      if (prev <> nil) then
        prev.next := cur.next
      else
        m_posLookup[h] := cur.next;
      break;
    end;
    prev := cur;
    cur := cur.next;
  end;

  // Remove connections to neighbour tiles.
  // Create connections with neighbour tiles.

  // Connect with layers in current tile.
  nneis := getTilesAt(tile.header.x, tile.header.y, @neis[0], MAX_NEIS);
  for j := 0 to nneis - 1 do
  begin
    if (neis[j] = tile) then continue;
    unconnectExtLinks(neis[j], tile);
  end;

  // Connect with neighbour tiles.
  for i := 0 to 7 do
  begin
    nneis := getNeighbourTilesAt(tile.header.x, tile.header.y, i, @neis[0], MAX_NEIS);
    for j := 0 to nneis - 1 do
      unconnectExtLinks(neis[j], tile);
  end;

  // Reset tile.
  if (tile.flags and DT_TILE_FREE_DATA <> 0) then
  begin
    // Owns data
    FreeMem(tile.data);
    tile.data := nil;
    tile.dataSize := 0;
    //todo: Doublecheck this
    if (data <> nil) then data^ := nil;
    if (dataSize <> nil) then dataSize^ := 0;
  end
  else
  begin
    //todo: Doublecheck this
    if (data <> nil) then data^ := tile.data;
    if (dataSize <> nil) then dataSize^ := tile.dataSize;
  end;

  tile.header := nil;
  tile.flags := 0;
  tile.linksFreeList := 0;
  tile.polys := nil;
  tile.verts := nil;
  tile.links := nil;
  tile.detailMeshes := nil;
  tile.detailVerts := nil;
  tile.detailTris := nil;
  tile.bvTree := nil;
  tile.offMeshCons := nil;

  // Update salt, salt should never be zero.
{$ifdef DT_POLYREF64}
  tile.salt := (tile.salt+1) and ((1<<DT_SALT_BITS)-1);
{$else}
  tile.salt := (tile.salt+1) and ((1 shl m_saltBits)-1);
{$endif}
  if (tile.salt = 0) then
    Inc(tile.salt);

  // Add to free list.
  tile.next := m_nextFree;
  m_nextFree := tile;

  Result := DT_SUCCESS;
end;

function TdtNavMesh.getTileRef(tile: PdtMeshTile): TdtTileRef;
var it: Cardinal;
begin
  if (tile = nil) then Exit(0);
  it := Cardinal(tile - m_tiles);
  Result := TdtTileRef(encodePolyId(tile.salt, it, 0));
end;

/// @par
///
/// Example use case:
/// @code
///
/// const dtPolyRef base := navmesh.getPolyRefBase(tile);
/// for i := 0 to tile.header.polyCount - 1 do
/// begin
///     const dtPoly* p := &tile.polys[i];
///     const dtPolyRef ref := base | (dtPolyRef)i;
///
///     // Use the reference to access the polygon data.
/// end;
/// @endcode
function TdtNavMesh.getPolyRefBase(tile: PdtMeshTile): TdtPolyRef;
var it: Cardinal;
begin
  if (tile = nil) then Exit(0);
  it := (tile - m_tiles);
  Result := encodePolyId(tile.salt, it, 0);
end;

type
PdtTileState = ^TdtTileState;
TdtTileState = record
  magic: Integer;                // Magic number, used to identify the data.
  version: Integer;              // Data version number.
  ref: TdtTileRef;              // Tile ref at the time of storing the data.
end;

PdtPolyState = ^TdtPolyState;
TdtPolyState = record
  flags: Word;            // Flags (see dtPolyFlags).
  area: Byte;              // Area ID of the polygon.
end;

///  @see #storeTileState
function TdtNavMesh.getTileStateSize(tile: PdtMeshTile): Integer;
var headerSize,polyStateSize: Integer;
begin
  if (tile = nil) then Exit(0);
  headerSize := dtAlign4(sizeof(TdtTileState));
  polyStateSize := dtAlign4(sizeof(TdtPolyState) * tile.header.polyCount);
  Result := headerSize + polyStateSize;
end;

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note The state data is only valid until the tile reference changes.
/// @see #getTileStateSize, #restoreTileState
function TdtNavMesh.storeTileState(tile: PdtMeshTile; data: PByte; maxDataSize: Integer): TdtStatus;
var sizeReq: Integer; tileState: PdtTileState; polyStates: PdtPolyState; i: Integer; p: PdtPoly; s: PdtPolyState;
begin
  // Make sure there is enough space to store the state.
  sizeReq := getTileStateSize(tile);
  if (maxDataSize < sizeReq) then
    Exit(DT_FAILURE or DT_BUFFER_TOO_SMALL);

  tileState := PdtTileState(data); Inc(data, dtAlign4(sizeof(TdtTileState)));
  polyStates := PdtPolyState(data); Inc(data, dtAlign4(sizeof(TdtPolyState) * tile.header.polyCount));

  // Store tile state.
  tileState.magic := DT_NAVMESH_STATE_MAGIC;
  tileState.version := DT_NAVMESH_STATE_VERSION;
  tileState.ref := getTileRef(tile);

  // Store per poly state.
  for i := 0 to tile.header.polyCount - 1 do
  begin
    p := @tile.polys[i];
    s := @polyStates[i];
    s.flags := p.flags;
    s.area := p.getArea();
  end;

  Result := DT_SUCCESS;
end;

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note This function does not impact the tile's #dtTileRef and #dtPolyRef's.
/// @see #storeTileState
function TdtNavMesh.restoreTileState(tile: PdtMeshTile; data: PByte; maxDataSize: Integer): TdtStatus;
var sizeReq: Integer; tileState: PdtTileState; polyStates: PdtPolyState; i: Integer; p: PdtPoly; s: PdtPolyState;
begin
  // Make sure there is enough space to store the state.
  sizeReq := getTileStateSize(tile);
  if (maxDataSize < sizeReq) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  tileState := PdtTileState(data); Inc(data, dtAlign4(sizeof(TdtTileState)));
  polyStates := PdtPolyState(data); Inc(data, dtAlign4(sizeof(TdtPolyState) * tile.header.polyCount));

  // Check that the restore is possible.
  if (tileState.magic <> DT_NAVMESH_STATE_MAGIC) then
    Exit(DT_FAILURE or DT_WRONG_MAGIC);
  if (tileState.version <> DT_NAVMESH_STATE_VERSION) then
    Exit(DT_FAILURE or DT_WRONG_VERSION);
  if (tileState.ref <> getTileRef(tile)) then
    Exit(DT_FAILURE or DT_INVALID_PARAM);

  // Restore per poly state.
  for i := 0 to tile.header.polyCount - 1 do
  begin
    p := @tile.polys[i];
    s := @polyStates[i];
    p.flags := s.flags;
    p.setArea(s.area);
  end;

  Result := DT_SUCCESS;
end;

/// @par
///
/// Off-mesh connections are stored in the navigation mesh as special 2-vertex
/// polygons with a single edge. At least one of the vertices is expected to be
/// inside a normal polygon. So an off-mesh connection is "entered" from a
/// normal polygon at one of its endpoints. This is the polygon identified by
/// the prevRef parameter.
function TdtNavMesh.getOffMeshConnectionPolyEndPoints(prevRef, polyRef: TdtPolyRef; startPos, endPos: PSingle): TdtStatus;
var salt, it, ip: Cardinal; tile: PdtMeshTile; poly: PdtPoly; idx0,idx1: Integer; i: Cardinal;
begin
  if (polyRef = 0) then
    Exit(DT_FAILURE);

  // Get current polygon
  decodePolyId(polyRef, @salt, @it, @ip);
  if (it >= m_maxTiles) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (m_tiles[it].salt <> salt) or (m_tiles[it].header = nil) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  tile := @m_tiles[it];
  if (ip >= tile.header.polyCount) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  poly := @tile.polys[ip];

  // Make sure that the current poly is indeed off-mesh link.
  if (poly.getType() <> DT_POLYTYPE_OFFMESH_CONNECTION) then
    Exit(DT_FAILURE);

  // Figure out which way to hand out the vertices.
  idx0 := 0; idx1 := 1;

  // Find link that points to first vertex.
  i := poly.firstLink;
  while (i <> DT_NULL_LINK) do
  begin
    if (tile.links[i].edge = 0) then
    begin
      if (tile.links[i].ref <> prevRef) then
      begin
        idx0 := 1;
        idx1 := 0;
      end;
      break;
    end;

    i := tile.links[i].next;
  end;

  dtVcopy(startPos, @tile.verts[poly.verts[idx0]*3]);
  dtVcopy(endPos, @tile.verts[poly.verts[idx1]*3]);

  Result := DT_SUCCESS;
end;


function TdtNavMesh.getOffMeshConnectionByRef(ref: TdtPolyRef): PdtOffMeshConnection;
var salt, it, ip: Cardinal; tile: PdtMeshTile; poly: PdtPoly; idx: Cardinal;
begin
  if (ref = 0) then
    Exit(nil);

  // Get current polygon
  decodePolyId(ref, @salt, @it, @ip);
  if (it >= m_maxTiles) then Exit(nil);
  if (m_tiles[it].salt <> salt) or (m_tiles[it].header = nil) then Exit(nil);
  tile := @m_tiles[it];
  if (ip >= tile.header.polyCount) then Exit(nil);
  poly := @tile.polys[ip];

  // Make sure that the current poly is indeed off-mesh link.
  if (poly.getType() <> DT_POLYTYPE_OFFMESH_CONNECTION) then
    Exit(nil);

  idx :=  ip - tile.header.offMeshBase;
  Assert(idx < tile.header.offMeshConCount);
  Result := @tile.offMeshCons[idx];
end;


function TdtNavMesh.setPolyFlags(ref: TdtPolyRef; flags: Word): TdtStatus;
var salt, it, ip: Cardinal; tile: PdtMeshTile; poly: PdtPoly;
begin
  if (ref = 0) then Exit(DT_FAILURE);
  decodePolyId(ref, @salt, @it, @ip);
  if (it >= m_maxTiles) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (m_tiles[it].salt <> salt) or (m_tiles[it].header = nil) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  tile := @m_tiles[it];
  if (ip >= tile.header.polyCount) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  poly := @tile.polys[ip];

  // Change flags.
  poly.flags := flags;

  Result := DT_SUCCESS;
end;

function TdtNavMesh.getPolyFlags(ref: TdtPolyRef; resultFlags: PWord): TdtStatus;
var salt, it, ip: Cardinal; tile: PdtMeshTile; poly: PdtPoly;
begin
  if (ref = 0) then Exit(DT_FAILURE);
  decodePolyId(ref, @salt, @it, @ip);
  if (it >= m_maxTiles) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (m_tiles[it].salt <> salt) or (m_tiles[it].header = nil) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  tile := @m_tiles[it];
  if (ip >= tile.header.polyCount) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  poly := @tile.polys[ip];

  resultFlags^ := poly.flags;

  Result := DT_SUCCESS;
end;

function TdtNavMesh.setPolyArea(ref: TdtPolyRef; area: Byte): TdtStatus;
var salt, it, ip: Cardinal; tile: PdtMeshTile; poly: PdtPoly;
begin
  if (ref = 0) then Exit(DT_FAILURE);
  decodePolyId(ref, @salt, @it, @ip);
  if (it >= m_maxTiles) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (m_tiles[it].salt <> salt) or (m_tiles[it].header = nil) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  tile := @m_tiles[it];
  if (ip >= tile.header.polyCount) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  poly := @tile.polys[ip];

  poly.setArea(area);

  Result := DT_SUCCESS;
end;

function TdtNavMesh.getPolyArea(ref: TdtPolyRef; resultArea: PByte): TdtStatus;
var salt, it, ip: Cardinal; tile: PdtMeshTile; poly: PdtPoly;
begin
  if (ref = 0) then Exit(DT_FAILURE);
  decodePolyId(ref, @salt, @it, @ip);
  if (it >= m_maxTiles) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  if (m_tiles[it].salt <> salt) or (m_tiles[it].header = nil) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  tile := @m_tiles[it];
  if (ip >= tile.header.polyCount) then Exit(DT_FAILURE or DT_INVALID_PARAM);
  poly := @tile.polys[ip];

  resultArea^ := poly.getArea();

  Result := DT_SUCCESS;
end;


function TdtNavMesh.encodePolyId(salt, it, ip: Cardinal): TdtPolyRef;
begin
{$ifdef DT_POLYREF64}
  Result := ((dtPolyRef)salt << (DT_POLY_BITS+DT_TILE_BITS)) | ((dtPolyRef)it << DT_POLY_BITS) | (dtPolyRef)ip;
{$else}
  Result := (TdtPolyRef(salt) shl (m_polyBits+m_tileBits)) or (TdtPolyRef(it) shl m_polyBits) or TdtPolyRef(ip);
{$endif}
end;

  /// Decodes a standard polygon reference.
  ///  @note This function is generally meant for internal use only.
  ///  @param[in]  ref   The polygon reference to decode.
  ///  @param[out]  salt  The tile's salt value.
  ///  @param[out]  it    The index of the tile.
  ///  @param[out]  ip    The index of the polygon within the tile.
  ///  @see #encodePolyId
procedure TdtNavMesh.decodePolyId(ref: TdtPolyRef; salt, it, ip: PCardinal);
var saltMask, tileMask, polyMask: TdtPolyRef;
begin
{$ifdef DT_POLYREF64}
  const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
  const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
  const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
  salt = (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
  it = (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
  ip = (unsigned int)(ref & polyMask);
{$else}
  saltMask := (TdtPolyRef(1) shl m_saltBits)-1;
  tileMask := (TdtPolyRef(1) shl m_tileBits)-1;
  polyMask := (TdtPolyRef(1) shl m_polyBits)-1;
  salt^ := ((ref shr (m_polyBits+m_tileBits)) and saltMask);
  it^ := ((ref shr m_polyBits) and tileMask);
  ip^ := (ref and polyMask);
{$endif}
end;

/// Extracts a tile's salt value from the specified polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]  ref    The polygon reference.
///  @see #encodePolyId
function TdtNavMesh.decodePolyIdSalt(ref: TdtPolyRef): Cardinal;
var saltMask: TdtPolyRef;
begin
{$ifdef DT_POLYREF64}
  const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
  Result := (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
{$else}
  saltMask := (TdtPolyRef(1) shl m_saltBits)-1;
  Result := Cardinal((ref shr (m_polyBits+m_tileBits)) and saltMask);
{$endif}
end;

/// Extracts the tile's index from the specified polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]  ref    The polygon reference.
///  @see #encodePolyId
function TdtNavMesh.decodePolyIdTile(ref: TdtPolyRef): Cardinal;
var tileMask: TdtPolyRef;
begin
{$ifdef DT_POLYREF64}
  const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
  Result := (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
{$else}
  tileMask := (TdtPolyRef(1) shl m_tileBits)-1;
  Result := Cardinal((ref shr m_polyBits) and tileMask);
{$endif}
end;

/// Extracts the polygon's index (within its tile) from the specified polygon reference.
///  @note This function is generally meant for internal use only.
///  @param[in]  ref    The polygon reference.
///  @see #encodePolyId
function TdtNavMesh.decodePolyIdPoly(ref: TdtPolyRef): Cardinal;
var polyMask: TdtPolyRef;
begin
{$ifdef DT_POLYREF64}
  const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
  Result := (unsigned int)(ref & polyMask);
{$else}
  polyMask := (TdtPolyRef(1) shl m_polyBits)-1;
  Result := Cardinal(ref and polyMask);
{$endif}
end;


procedure TdtNavMesh.SaveToStream(aStream: TMemoryStream);
var
  I: Integer;
  mt: PdtMeshTile;
  mh: TdtMeshHeader;
  //eol: Word;
begin
  //eol := $0D0A;
  aStream.Write(m_params, SizeOf(m_params));
  aStream.Write(m_orig[0], SizeOf(m_orig));

  aStream.Write(m_tileWidth, SizeOf(m_tileWidth));
  aStream.Write(m_tileHeight, SizeOf(m_tileHeight));
  aStream.Write(m_maxTiles, SizeOf(m_maxTiles));
  aStream.Write(m_tileLutSize, SizeOf(m_tileLutSize));
  aStream.Write(m_tileLutMask, SizeOf(m_tileLutMask));

  //m_posLookup: PPointer;
  for I := 0 to m_tileLutSize - 1 do
  begin
    aStream.Write(I, SizeOf(Integer));
    if m_posLookup[I] <> nil then
    begin
      mt := PdtMeshTile(m_posLookup[I]);
      aStream.Write(mt.salt, SizeOf(mt.salt));
      aStream.Write(mt.linksFreeList, SizeOf(mt.linksFreeList));
      // ..
    end;
  end;

  //m_nextFree: PdtMeshTile;
  if m_nextFree <> nil then
  begin
    aStream.Write(m_nextFree.salt, SizeOf(m_nextFree.salt));
    aStream.Write(m_nextFree.linksFreeList, SizeOf(m_nextFree.linksFreeList));
    // ..
  end;

  //m_tiles: PdtMeshTile;
  for I := 0 to m_tileLutSize - 1 do
  begin
    mt := @m_tiles[I];
    aStream.Write(mt.salt, SizeOf(mt.salt));
    aStream.Write(mt.linksFreeList, SizeOf(mt.linksFreeList));

    mh := mt.header^;
    aStream.Write(mh, SizeOf(mh));

    aStream.Write(mt.polys^, SizeOf(TdtPoly) * mh.polyCount);
    aStream.Write(mt.verts^, SizeOf(Single) * 3 * mh.vertCount);
    aStream.Write(mt.links^, SizeOf(TdtLink) * mh.maxLinkCount);
    aStream.Write(mt.detailMeshes^, SizeOf(TdtPolyDetail) * mh.detailMeshCount);
    aStream.Write(mt.detailVerts^, SizeOf(Single) * 3 * mh.detailVertCount);
    aStream.Write(mt.detailTris^, SizeOf(Byte) * 4 * mh.detailTriCount);
    aStream.Write(mt.bvTree^, SizeOf(TdtBVNode) * mh.bvNodeCount);
    aStream.Write(mt.offMeshCons^, SizeOf(TdtOffMeshConnection) * mh.offMeshConCount);
    aStream.Write(mt.data^, mt.dataSize);
    aStream.Write(mt.dataSize, SizeOf(mt.dataSize));
    aStream.Write(mt.flags, SizeOf(mt.flags));

    //next: PdtMeshTile;
    if mt.next <> nil then
    begin
      aStream.Write(mt.next.salt, SizeOf(mt.next.salt));
      aStream.Write(mt.next.linksFreeList, SizeOf(mt.next.linksFreeList));
    end;
  end;

  {$ifndef DT_POLYREF64}
    aStream.Write(m_saltBits, SizeOf(m_saltBits));
    aStream.Write(m_tileBits, SizeOf(m_tileBits));
    aStream.Write(m_polyBits, SizeOf(m_polyBits));
  {$endif}
end;


end.
