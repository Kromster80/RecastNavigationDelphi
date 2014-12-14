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
unit RN_DetourPathQueue;
interface
uses RN_DetourNavMesh, RN_DetourNavMeshHelper, RN_DetourNavMeshQuery, RN_DetourStatus;

const DT_PATHQ_INVALID = 0;

type
  TdtPathQueueRef = Cardinal;

  TdtPathQueue = class
  private
    type
    PPathQuery = ^TPathQuery;
    TPathQuery = record
      ref: TdtPathQueueRef;
      /// Path find start and end location.
      startPos, endPos: array [0..2] of Single;
      startRef, endRef: TdtPolyRef;
      /// Result.
      path: PdtPolyRef;
      npath: Integer;
      /// State.
      status: TdtStatus;
      keepAlive: Integer;
      filter: TdtQueryFilter; ///< TODO: This is potentially dangerous!
    end;

    const MAX_QUEUE = 8;
    var
    m_queue: array [0..MAX_QUEUE-1] of TPathQuery;
    m_nextHandle: TdtPathQueueRef;
    m_maxPathSize: Integer;
    m_queueHead: Integer;
    m_navquery: TdtNavMeshQuery;

    procedure purge();
  public
    constructor Create;
    destructor Destroy; override;

    function init(const maxPathSize, maxSearchNodeCount: Integer; nav: TdtNavMesh): Boolean;

    procedure update(const maxIters: Integer);

    function request(startRef, endRef: TdtPolyRef;
                 const startPos, endPos: PSingle;
                 const filter: TdtQueryFilter): TdtPathQueueRef;

    function getRequestStatus(ref: TdtPathQueueRef): TdtStatus;

    function getPathResult(ref: TdtPathQueueRef; path: PdtPolyRef; pathSize: PInteger; const maxPath: Integer): TdtStatus;

    function getNavQuery(): TdtNavMeshQuery; { return m_navquery; }
  end;

implementation
uses RN_DetourCommon;


constructor TdtPathQueue.Create;
var i: Integer;
begin
  inherited;

	m_nextHandle := 1;
	m_maxPathSize := 0;
	m_queueHead := 0;
	m_navquery := nil;

	for i := 0 to MAX_QUEUE - 1 do
		m_queue[i].path := nil;
end;

destructor TdtPathQueue.Destroy;
begin
	purge();

  inherited;
end;

procedure TdtPathQueue.purge();
var i: Integer;
begin
	dtFreeNavMeshQuery(m_navquery);
	m_navquery := nil;
	for i := 0 to MAX_QUEUE - 1 do
	begin
		FreeMem(m_queue[i].path);
		m_queue[i].path := nil;
	end;
end;

function TdtPathQueue.init(const maxPathSize, maxSearchNodeCount: Integer; nav: TdtNavMesh): Boolean;
var i: Integer;
begin
	purge();

	m_navquery := dtAllocNavMeshQuery();
	if (m_navquery = nil) then
		Exit(false);
	if (dtStatusFailed(m_navquery.init(nav, maxSearchNodeCount))) then
		Exit(false);
	
	m_maxPathSize := maxPathSize;
	for i := 0 to MAX_QUEUE - 1 do
	begin
		m_queue[i].ref := DT_PATHQ_INVALID;
		GetMem(m_queue[i].path, sizeof(TdtPolyRef)*m_maxPathSize);
		if (m_queue[i].path = nil) then
			Exit(false);
	end;

	m_queueHead := 0;

	Result := true;
end;

procedure TdtPathQueue.update(const maxIters: Integer);
const MAX_KEEP_ALIVE = 2; // in update ticks.
var iterCount, i, iters: Integer; q: PPathQuery;
begin
	// Update path request until there is nothing to update
	// or upto maxIters pathfinder iterations has been consumed.
	iterCount := maxIters;

	for i := 0 to MAX_QUEUE - 1 do
	begin
		q := @m_queue[m_queueHead mod MAX_QUEUE];
		
		// Skip inactive requests.
		if (q.ref = DT_PATHQ_INVALID) then
		begin
			Inc(m_queueHead);
			continue;
		end;
		
		// Handle completed request.
		if (dtStatusSucceed(q.status) or dtStatusFailed(q.status)) then
		begin
			// If the path result has not been read in few frames, free the slot.
			Inc(q.keepAlive);
			if (q.keepAlive > MAX_KEEP_ALIVE) then
			begin
				q.ref := DT_PATHQ_INVALID;
				q.status := 0;
			end;
			
			Inc(m_queueHead);
			continue;
		end;
		
		// Handle query start.
		if (q.status = 0) then
		begin
			q.status := m_navquery.initSlicedFindPath(q.startRef, q.endRef, @q.startPos, @q.endPos, q.filter);
		end;
		// Handle query in progress.
		if (dtStatusInProgress(q.status)) then
		begin
			iters := 0;
			q.status := m_navquery.updateSlicedFindPath(iterCount, @iters);
			Dec(iterCount, iters);
		end;
		if (dtStatusSucceed(q.status)) then
		begin
			q.status := m_navquery.finalizeSlicedFindPath(q.path, @q.npath, m_maxPathSize);
		end;

		if (iterCount <= 0) then
			break;

		Inc(m_queueHead);
	end;
end;

function TdtPathQueue.request(startRef, endRef: TdtPolyRef;
                 const startPos, endPos: PSingle;
                 const filter: TdtQueryFilter): TdtPathQueueRef;
var slot, i: Integer; ref: TdtPathQueueRef; q: PPathQuery;
begin
	// Find empty slot
	slot := -1;
	for i := 0 to MAX_QUEUE - 1 do
	begin
		if (m_queue[i].ref = DT_PATHQ_INVALID) then
		begin
			slot := i;
			break;
		end;
	end;
	// Could not find slot.
	if (slot = -1) then
		Exit(DT_PATHQ_INVALID);
	
	ref := m_nextHandle;
  Inc(m_nextHandle);
	if (m_nextHandle = DT_PATHQ_INVALID) then Inc(m_nextHandle);
	
	q := @m_queue[slot];
	q.ref := ref;
	dtVcopy(@q.startPos, startPos);
	q.startRef := startRef;
	dtVcopy(@q.endPos, endPos);
	q.endRef := endRef;

	q.status := 0;
	q.npath := 0;
	q.filter := filter;
	q.keepAlive := 0;
	
	Result := ref;
end;

function TdtPathQueue.getRequestStatus(ref: TdtPathQueueRef): TdtStatus;
var i: Integer;
begin
	for i := 0 to MAX_QUEUE - 1 do
	begin
		if (m_queue[i].ref = ref) then
			Exit(m_queue[i].status);
	end;
	Result := DT_FAILURE;
end;

function TdtPathQueue.getPathResult(ref: TdtPathQueueRef; path: PdtPolyRef; pathSize: PInteger; const maxPath: Integer): TdtStatus;
var i, n: Integer; q: PPathQuery; details: TdtStatus;
begin
	for i := 0 to MAX_QUEUE - 1 do
	begin
		if (m_queue[i].ref = ref) then
		begin
			q := @m_queue[i];
			details := q.status and DT_STATUS_DETAIL_MASK;
			// Free request for reuse.
			q.ref := DT_PATHQ_INVALID;
			q.status := 0;
			// Copy path
			n := dtMin(q.npath, maxPath);
			Move(q.path^, path^, sizeof(TdtPolyRef)*n);
			pathSize^ := n;
			Exit(details or DT_SUCCESS);
		end;
	end;
	Result := DT_FAILURE;
end;


function TdtPathQueue.getNavQuery(): TdtNavMeshQuery; begin Result := m_navquery; end;

end.