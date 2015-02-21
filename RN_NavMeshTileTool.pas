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

unit RN_NavMeshTileTool;
interface
uses
  Classes, Controls, Math, OpenGL, StrUtils, SysUtils, Unit_FrameTileTool,
  RN_InputGeom, RN_Sample, RN_SampleTileMesh, RN_DetourNavMesh, RN_Recast, RN_ChunkyTriMesh;

type
  TNavMeshTileTool = class(TSampleTool)
  private
    fFrame: TFrameTileTool;
    fUpdateUI: Boolean;
  public
    m_sample: TSample_TileMesh;
    m_hitPos: array [0..2] of Single;
    m_hitPosSet: Boolean;
  public
    constructor Create(aOwner: TWinControl);
    destructor Destroy; override;

    procedure init(sample: TSample); override;

    procedure reset(); override;

    procedure handleMenu(Sender: TObject); override;

    procedure handleClick(s,p: PSingle; shift: Boolean); override;

    procedure handleToggle(); override;

    procedure handleStep(); override;

    procedure handleUpdate(dt: Single); override;

    procedure handleRender(); override;

    procedure handleRenderOverlay(proj, model: PDouble; view: PInteger); override;
  end;


implementation
uses
  RN_RecastDebugDraw, RN_RecastHelper, RN_DetourNavMeshBuilder, RN_DetourDebugDraw, RN_NavMeshTesterTool, RN_NavMeshPruneTool,
  {RN_OffMeshConnectionTool, RN_ConvexVolumeTool,} RN_CrowdTool;


constructor TNavMeshTileTool.Create(aOwner: TWinControl);
begin
  inherited Create;

  &type := TOOL_TILE_EDIT;

  fFrame := TFrameTileTool.Create(aOwner);
  fFrame.Align := alClient;
  fFrame.Parent := aOwner;
  fFrame.Visible := True;
  fFrame.btnCreateAll.OnClick := handleMenu;
  fFrame.btnRemoveAll.OnClick := handleMenu;
end;

destructor TNavMeshTileTool.Destroy;
begin
  fFrame.Free;
  inherited;
end;

procedure TNavMeshTileTool.init(sample: TSample);
begin
  m_sample := TSample_TileMesh(sample);
end;

procedure TNavMeshTileTool.reset();
begin

end;

procedure TNavMeshTileTool.handleMenu(Sender: TObject);
begin
  if fUpdateUI then Exit;

  if Sender = fFrame.btnCreateAll then
  begin
    if (m_sample <> nil) then
      m_sample.buildAllTiles();
  end;

  if Sender = fFrame.btnRemoveAll then
  begin
    if (m_sample <> nil) then
      m_sample.removeAllTiles();
  end;
end;

procedure TNavMeshTileTool.handleClick(s,p: PSingle; shift: Boolean);
begin
  m_hitPosSet := true;
  rcVcopy(@m_hitPos[0], p);
  if (m_sample <> nil) then
  begin
    if (shift) then
      m_sample.removeTile(@m_hitPos[0])
    else
      m_sample.buildTile(@m_hitPos[0]);
  end;
end;

procedure TNavMeshTileTool.handleToggle();
begin

end;

procedure TNavMeshTileTool.handleStep();
begin

end;

procedure TNavMeshTileTool.handleUpdate(dt: Single);
begin

end;

procedure TNavMeshTileTool.handleRender();
var s: Single;
begin
  if (m_hitPosSet) then
  begin
    s := m_sample.getAgentRadius;
    glColor4ub(0,0,0,128);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    glVertex3f(m_hitPos[0]-s,m_hitPos[1]+0.1,m_hitPos[2]);
    glVertex3f(m_hitPos[0]+s,m_hitPos[1]+0.1,m_hitPos[2]);
    glVertex3f(m_hitPos[0],m_hitPos[1]-s+0.1,m_hitPos[2]);
    glVertex3f(m_hitPos[0],m_hitPos[1]+s+0.1,m_hitPos[2]);
    glVertex3f(m_hitPos[0],m_hitPos[1]+0.1,m_hitPos[2]-s);
    glVertex3f(m_hitPos[0],m_hitPos[1]+0.1,m_hitPos[2]+s);
    glEnd();
    glLineWidth(1.0);
  end;
end;

procedure TNavMeshTileTool.handleRenderOverlay(proj, model: PDouble; view: PInteger);
//var x, y, z: GLdouble;
begin
    {if (m_hitPosSet and gluProject((GLdouble)m_hitPos[0], (GLdouble)m_hitPos[1], (GLdouble)m_hitPos[2],
                    model, proj, view, &x, &y, &z))
    begin
      int tx=0, ty=0;
      m_sample->getTilePos(m_hitPos, tx, ty);
      char text[32];
      snprintf(text,32,"(%d,%d)", tx,ty);
      imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
    end;

    // Tool help
    const int h = view[3]: array [0..2] of Integer;
    imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB: Rebuild hit tile.  Shift+LMB: Clear hit tile.", imguiRGBA(255,255,255,192));
  end;}
end;

end.
