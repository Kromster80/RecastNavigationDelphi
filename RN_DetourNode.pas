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

unit RN_DetourNode;
interface
uses RN_DetourNavMeshHelper;

const
//enum dtNodeFlags
  DT_NODE_OPEN = $01;
  DT_NODE_CLOSED = $02;
  DT_NODE_PARENT_DETACHED = $04; // parent of the node is not adjacent. Found using raycast.

type TdtNodeIndex = Word; PdtNodeIndex = ^TdtNodeIndex;
const DT_NULL_IDX: TdtNodeIndex = $ffff; // Inverse of 0

type
  PdtNode = ^TdtNode;
  TdtNode = record
    pos: array [0..2] of Single;        ///< Position of the node.
    cost: Single;          ///< Cost from previous node to current node.
    total: Single;        ///< Cost up to the node.
    pidx: Cardinal;    ///< Index to parent node.
    state: Byte;    ///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
    flags: Byte;    ///< Node flags. A combination of dtNodeFlags.
    id: TdtPolyRef;        ///< Polygon ref the node corresponds to.
  end;


const DT_MAX_STATES_PER_NODE = 4;  // number of extra states per node. See dtNode::state

type
  TdtNodePool = class
  private
    m_nodes: PdtNode;
    m_first: PdtNodeIndex;
    m_next: PdtNodeIndex;
    m_maxNodes: Integer;
    m_hashSize: Integer;
    m_nodeCount: Integer;
  public
    constructor Create(maxNodes, hashSize: Integer);
    destructor Destroy; override;
    //inline void operator=(const dtNodePool&) {}
    procedure clear();

    // Get a dtNode by ref and extra state information. If there is none then - allocate
    // There can be more than one node for the same polyRef but with different extra state information
    function getNode(id: TdtPolyRef; state: Byte = 0): PdtNode;
    function findNode(id: TdtPolyRef; state: Byte): PdtNode;
    function findNodes(id: TdtPolyRef; var nodes: array of PdtNode; maxNodes: Integer): Cardinal;

    function getNodeIdx(node: PdtNode): Cardinal;
    function getNodeAtIdx(idx: Cardinal): PdtNode;
    function getMemUsed(): Integer;

    property getMaxNodes: Integer read m_maxNodes;

    property getHashSize: Integer read m_hashSize;
    function getFirst(bucket: Integer): TdtNodeIndex;
    function getNext(i: Integer): TdtNodeIndex;
    property getNodeCount: Integer read m_nodeCount;
  end;

  TdtNodeQueue = class
  private
    m_heap: array of PdtNode;
    m_capacity: Integer;
    m_size: Integer;
    procedure bubbleUp(i: Integer; node: PdtNode);
    procedure trickleDown(i: Integer; node: PdtNode);
  public
    constructor Create(n: Integer);
    destructor Destroy; override;
    //inline void operator=(dtNodeQueue&) {}

    procedure clear();
    function top(): PdtNode;
    function pop(): PdtNode;
    procedure push(node: PdtNode);
    procedure modify(node: PdtNode);
    function empty(): Boolean;
    function getMemUsed(): Integer;

    property getCapacity: Integer read m_capacity;
  end;

implementation
uses RN_DetourCommon;

{$ifdef DT_POLYREF64}
// From Thomas Wang, https://gist.github.com/badboy/6267743
function dtHashRef(a: TdtPolyRef): Cardinal;
begin
  a := (not a) + (a shl 18); // a = (a << 18) - a - 1;
  a := a xor (a shr 31);
  a := a * 21; // a = (a + (a << 2)) + (a << 4);
  a := a xor (a shr 11);
  a := a + (a shl 6);
  a := a xor (a shr 22);
  Result := Cardinal(a);
end
{$else}
function dtHashRef(a: TdtPolyRef): Cardinal;
begin
{$Q-}
  a := a + not (a shl 15);
  a := a xor (a shr 10);
  a := a + (a shl 3);
  a := a xor (a shr 6);
  a := a + not (a shl 11);
  a := a xor (a shr 16);
  Result := Cardinal(a);
{$Q+}
end;
{$endif}

//////////////////////////////////////////////////////////////////////////////////////////
constructor TdtNodePool.Create(maxNodes, hashSize: Integer);
begin
  inherited Create;
  m_maxNodes := maxNodes;
  m_hashSize := hashSize;

  Assert(dtNextPow2(m_hashSize) = m_hashSize);
  Assert(m_maxNodes > 0);

  GetMem(m_nodes, sizeof(TdtNode)*m_maxNodes);
  GetMem(m_next, sizeof(TdtNodeIndex)*m_maxNodes);
  GetMem(m_first, sizeof(TdtNodeIndex)*hashSize);

  //dtAssert(m_nodes);
  //dtAssert(m_next);
  //dtAssert(m_first);

  FillChar(m_first[0], sizeof(TdtNodeIndex)*m_hashSize, $ff);
  FillChar(m_next[0], sizeof(TdtNodeIndex)*m_maxNodes, $ff);
end;

destructor TdtNodePool.Destroy;
begin
  FreeMem(m_nodes);
  FreeMem(m_next);
  FreeMem(m_first);

  inherited;
end;

procedure TdtNodePool.clear();
begin
  FillChar(m_first[0], sizeof(TdtNodeIndex)*m_hashSize, $ff);
  m_nodeCount := 0;
end;

function TdtNodePool.findNodes(id: TdtPolyRef; var nodes: array of PdtNode; maxNodes: Integer): Cardinal;
var n: Integer; bucket: Cardinal; i: TdtNodeIndex;
begin
  n := 0;
  bucket := dtHashRef(id) and (m_hashSize-1);
  i := m_first[bucket];
  while (i <> DT_NULL_IDX) do
  begin
    if (m_nodes[i].id = id) then
    begin
      if (n >= maxNodes) then
        Exit(n);
      nodes[n] := @m_nodes[i];
      Inc(n);
    end;
    i := m_next[i];
  end;

  Result := n;
end;

function TdtNodePool.findNode(id: TdtPolyRef; state: Byte): PdtNode;
var bucket: Cardinal; i: TdtNodeIndex;
begin
  bucket := dtHashRef(id) and (m_hashSize-1);
  i := m_first[bucket];
  while (i <> DT_NULL_IDX) do
  begin
    if (m_nodes[i].id = id) and (m_nodes[i].state = state) then
      Exit(@m_nodes[i]);
    i := m_next[i];
  end;
  Result := nil;
end;

function TdtNodePool.getNode(id: TdtPolyRef; state: Byte = 0): PdtNode;
var bucket: Cardinal; i: TdtNodeIndex; node: PdtNode;
begin
  bucket := dtHashRef(id) and (m_hashSize-1);
  i := m_first[bucket];
  node := nil;
  while (i <> DT_NULL_IDX) do
  begin
    if (m_nodes[i].id = id) and (m_nodes[i].state = state) then
      Exit(@m_nodes[i]);
    i := m_next[i];
  end;

  if (m_nodeCount >= m_maxNodes) then
    Exit(nil);

  i := TdtNodeIndex(m_nodeCount);
  Inc(m_nodeCount);

  // Init node
  node := @m_nodes[i];
  node.pidx := 0;
  node.cost := 0;
  node.total := 0;
  node.id := id;
  node.state := state;
  node.flags := 0;

  m_next[i] := m_first[bucket];
  m_first[bucket] := i;

  Result := node;
end;


function TdtNodePool.getNodeIdx(node: PdtNode): Cardinal;
begin
  if (node = nil) then Exit(0);
  Result := Cardinal(node - m_nodes)+1;
end;

function TdtNodePool.getNodeAtIdx(idx: Cardinal): PdtNode;
begin
  if (idx = 0) then Exit(nil);
  Result := @m_nodes[idx-1];
end;

function TdtNodePool.getMemUsed(): Integer;
begin
  Result := sizeof(Self) +
    sizeof(TdtNode)*m_maxNodes +
    sizeof(TdtNodeIndex)*m_maxNodes +
    sizeof(TdtNodeIndex)*m_hashSize;
end;

function TdtNodePool.getFirst(bucket: Integer): TdtNodeIndex; begin Result := m_first[bucket]; end;
function TdtNodePool.getNext(i: Integer): TdtNodeIndex; begin Result := m_next[i]; end;

//////////////////////////////////////////////////////////////////////////////////////////
procedure TdtNodeQueue.clear();
begin
  m_size := 0;
end;

function TdtNodeQueue.top(): PdtNode;
begin
  Result := m_heap[0];
end;

function TdtNodeQueue.pop(): PdtNode;
var reslt: PdtNode;
begin
  reslt := m_heap[0];
  Dec(m_size);
  trickleDown(0, m_heap[m_size]);
  Result := reslt;
end;

procedure TdtNodeQueue.push(node: PdtNode);
begin
  Inc(m_size);
  bubbleUp(m_size-1, node);
end;

procedure TdtNodeQueue.modify(node: PdtNode);
var i: Integer;
begin
  for i := 0 to m_size - 1 do
  begin
    if (m_heap[i] = node) then
    begin
      bubbleUp(i, node);
      Exit;
    end;
  end;
end;

function TdtNodeQueue.empty(): Boolean; begin Result := m_size = 0; end;

function TdtNodeQueue.getMemUsed(): Integer;
begin
  Result := sizeof(Self) +
  sizeof(PdtNode)*(m_capacity+1);
end;

constructor TdtNodeQueue.Create(n: Integer);
begin
  m_capacity := n;

  Assert(m_capacity > 0);

  SetLength(m_heap, (m_capacity+1));
  //dtAssert(m_heap);
end;

destructor TdtNodeQueue.Destroy;
begin
  //dtFree(m_heap); //Delphi: array disposal is automatic
  inherited;
end;

procedure TdtNodeQueue.bubbleUp(i: Integer; node: PdtNode);
var parent: Integer;
begin
  parent := (i-1) div 2;
  // note: (index > 0) means there is a parent
  while ((i > 0) and (m_heap[parent].total > node.total)) do
  begin
    m_heap[i] := m_heap[parent];
    i := parent;
    parent := (i-1) div 2;
  end;
  m_heap[i] := node;
end;

procedure TdtNodeQueue.trickleDown(i: Integer; node: PdtNode);
var child: Integer;
begin
  child := (i*2)+1;
  while (child < m_size) do
  begin
    if (((child+1) < m_size) and
      (m_heap[child].total > m_heap[child+1].total)) then
    begin
      Inc(child);
    end;
    m_heap[i] := m_heap[child];
    i := child;
    child := (i*2)+1;
  end;
  bubbleUp(i, node);
end;

end.
