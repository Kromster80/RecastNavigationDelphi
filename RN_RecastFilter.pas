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

unit RN_RecastFilter;
interface
uses
  Math, SysUtils, RN_Helper, RN_Recast;

/// Marks non-walkable spans as walkable if their maximum is within @p walkableClimp of a walkable neihbor.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in]    walkableClimb  Maximum ledge height that is considered to still be traversable.
///                  [Limit: >=0] [Units: vx]
///  @param[in,out]  solid      A fully built heightfield.  (All spans have been added.)
procedure rcFilterLowHangingWalkableObstacles(ctx: TrcContext; const walkableClimb: Integer; const solid: TrcHeightfield);

/// Marks spans that are ledges as not-walkable.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in]    walkableHeight  Minimum floor to 'ceiling' height that will still allow the floor area to
///                  be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[in]    walkableClimb  Maximum ledge height that is considered to still be traversable.
///                  [Limit: >=0] [Units: vx]
///  @param[in,out]  solid      A fully built heightfield.  (All spans have been added.)
procedure rcFilterLedgeSpans(ctx: TrcContext; const walkableHeight: Integer;
            const walkableClimb: Integer; const solid: TrcHeightfield);

/// Marks walkable spans as not walkable if the clearence above the span is less than the specified height.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in]    walkableHeight  Minimum floor to 'ceiling' height that will still allow the floor area to
///                  be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[in,out]  solid      A fully built heightfield.  (All spans have been added.)
procedure rcFilterWalkableLowHeightSpans(ctx: TrcContext; walkableHeight: Integer; const solid: TrcHeightfield);


implementation
uses RN_RecastHelper;

/// @par
///
/// Allows the formation of walkable regions that will flow over low lying
/// objects such as curbs, and up structures such as stairways.
///
/// Two neighboring spans are walkable if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) < waklableClimb</tt>
///
/// @warning Will override the effect of #rcFilterLedgeSpans.  So if both filters are used, call
/// #rcFilterLedgeSpans after calling this filter.
///
/// @see rcHeightfield, rcConfig
procedure rcFilterLowHangingWalkableObstacles(ctx: TrcContext; const walkableClimb: Integer; const solid: TrcHeightfield);
var w,h,x,y: Integer; ps,s: PrcSpan; previousWalkable,walkable: Boolean; previousArea: Byte;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_FILTER_LOW_OBSTACLES);

  w := solid.width;
  h := solid.height;

  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      ps := nil;
      previousWalkable := false;
      previousArea := RC_NULL_AREA;

      //for (rcSpan* s := solid.spans[x + y*w]; s; ps := s, s := s.next)
      s := solid.spans[x + y*w];
      while (s <> nil) do
      begin
        walkable := s.area <> RC_NULL_AREA;
        // If current span is not walkable, but there is walkable
        // span just below it, mark the span above it walkable too.
        if (not walkable and previousWalkable) then
        begin
          if (Abs(s.smax - ps.smax) <= walkableClimb) then
            s.area := previousArea;
        end;
        // Copy walkable flag so that it cannot propagate
        // past multiple non-walkable objects.
        previousWalkable := walkable;
        previousArea := s.area;

        ps := s;
        s := s.next;
      end;
    end;
  end;

  ctx.stopTimer(RC_TIMER_FILTER_LOW_OBSTACLES);
end;

/// @par
///
/// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
/// from the current span's maximum.
/// This method removes the impact of the overestimation of conservative voxelization
/// so the resulting mesh will not have regions hanging in the air over ledges.
///
/// A span is a ledge if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
///
/// @see rcHeightfield, rcConfig
procedure rcFilterLedgeSpans(ctx: TrcContext; const walkableHeight: Integer;
            const walkableClimb: Integer; const solid: TrcHeightfield);
const MAX_HEIGHT = $ffff;
var w,h,x,y: Integer; s,ns: PrcSpan; bot,top,minh,asmin,asmax,dir,dx,dy,nbot,ntop: Integer;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_FILTER_BORDER);

  w := solid.width;
  h := solid.height;

  // Mark border spans.
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      //for (rcSpan* s := solid.spans[x + y*w]; s; s := s.next)
      s := solid.spans[x + y*w];
      while (s <> nil) do
      begin
        // Skip non walkable spans.
        if (s.area = RC_NULL_AREA) then
        begin
          //C++ seems to be doing loop increase, so do we
          s := s.next;
          continue;
        end;

        bot := (s.smax);
        if s.next <> nil then top := (s.next.smin) else top := MAX_HEIGHT;

        // Find neighbours minimum height.
        minh := MAX_HEIGHT;

        // Min and max height of accessible neighbours.
        asmin := s.smax;
        asmax := s.smax;

        for dir := 0 to 3 do
        begin
          dx := x + rcGetDirOffsetX(dir);
          dy := y + rcGetDirOffsetY(dir);
          // Skip neighbours which are out of bounds.
          if (dx < 0) or (dy < 0) or (dx >= w) or (dy >= h) then
          begin
            minh := rcMin(minh, -walkableClimb - bot);
            continue;
          end;

          // From minus infinity to the first span.
          ns := solid.spans[dx + dy*w];
          nbot := -walkableClimb;
          if ns <> nil then ntop := ns.smin else ntop := MAX_HEIGHT;
          // Skip neightbour if the gap between the spans is too small.
          if (rcMin(top,ntop) - rcMax(bot,nbot) > walkableHeight) then
            minh := rcMin(minh, nbot - bot);

          // Rest of the spans.
          //for (ns := solid.spans[dx + dy*w]; ns; ns := ns.next)
          ns := solid.spans[dx + dy*w];
          while (ns <> nil) do
          begin
            nbot := ns.smax;
            if ns.next <> nil then ntop := ns.next.smin else ntop := MAX_HEIGHT;
            // Skip neightbour if the gap between the spans is too small.
            if (rcMin(top,ntop) - rcMax(bot,nbot) > walkableHeight) then
            begin
              minh := rcMin(minh, nbot - bot);

              // Find min/max accessible neighbour height.
              if (Abs(nbot - bot) <= walkableClimb) then
              begin
                if (nbot < asmin) then asmin := nbot;
                if (nbot > asmax) then asmax := nbot;
              end;

            end;
            ns := ns.next;
          end;
        end;

        // The current span is close to a ledge if the drop to any
        // neighbour span is less than the walkableClimb.
        if (minh < -walkableClimb) then
          s.area := RC_NULL_AREA;

        // If the difference between all neighbours is too large,
        // we are at steep slope, mark the span as ledge.
        if ((asmax - asmin) > walkableClimb) then
        begin
          s.area := RC_NULL_AREA;
        end;

        s := s.next;
      end;
    end;
  end;

  ctx.stopTimer(RC_TIMER_FILTER_BORDER);
end;

/// @par
///
/// For this filter, the clearance above the span is the distance from the span's
/// maximum to the next higher span's minimum. (Same grid column.)
///
/// @see rcHeightfield, rcConfig
procedure rcFilterWalkableLowHeightSpans(ctx: TrcContext; walkableHeight: Integer; const solid: TrcHeightfield);
const MAX_HEIGHT = $ffff;
var w,h,x,y: Integer; s: PrcSpan; bot,top: Integer;
begin
  Assert(ctx <> nil);

  ctx.startTimer(RC_TIMER_FILTER_WALKABLE);

  w := solid.width;
  h := solid.height;

  // Remove walkable flag from spans which do not have enough
  // space above them for the agent to stand there.
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      s := solid.spans[x + y*w];
      while (s <> nil) do
      begin
        bot := (s.smax);
        if s.next <> nil then top := (s.next.smin) else top := MAX_HEIGHT;
        if ((top - bot) <= walkableHeight) then
          s.area := RC_NULL_AREA;

        s := s.next;
      end;
    end;
  end;

  ctx.stopTimer(RC_TIMER_FILTER_WALKABLE);
end;

end.
