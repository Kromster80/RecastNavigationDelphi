unit Unit_Form;
interface
uses
  Windows, Messages, SysUtils, Classes, Graphics, Controls, ComCtrls, Forms, StrUtils,
  StdCtrls, ExtCtrls, CheckLst, Spin, Math, Dialogs, dglOpenGL, KromOGLUtils,
  RN_InputGeom, RN_Recast, RN_SampleInterfaces, RN_SampleSoloMesh;

type
  TForm1 = class(TForm)
    Panel3: TPanel;
    Timer1: TTimer;
    Button1: TButton;
    rgDrawMode: TRadioGroup;
    Memo1: TMemo;
    GroupBox1: TGroupBox;
    seCellSize: TSpinEdit;
    seMaxSampleError: TSpinEdit;
    seSampleDistance: TSpinEdit;
    seVertsPerPoly: TSpinEdit;
    seMaxEdgeError: TSpinEdit;
    seMaxEdgeLength: TSpinEdit;
    rgPartitioning: TRadioGroup;
    seMergedRegionSize: TSpinEdit;
    seMinRegionSize: TSpinEdit;
    seMaxSlope: TSpinEdit;
    seMaxClimb: TSpinEdit;
    seAgentRadius: TSpinEdit;
    seAgentHeight: TSpinEdit;
    seCellHeight: TSpinEdit;
    chkKeepIntermediateResults: TCheckBox;
    rgInputMesh: TRadioGroup;
    rgChooseSample: TRadioGroup;
    CheckBox2: TCheckBox;
    CheckBox1: TCheckBox;
    Label13: TLabel;
    Label12: TLabel;
    Label11: TLabel;
    Label10: TLabel;
    Label9: TLabel;
    Label8: TLabel;
    Label7: TLabel;
    Label6: TLabel;
    Label5: TLabel;
    Label4: TLabel;
    Label3: TLabel;
    Label2: TLabel;
    Label1: TLabel;
    rgTool: TRadioGroup;
    gbTool: TGroupBox;
    Panel1: TPanel;
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormResize(Sender: TObject);
    procedure DoIdle(Sender: TObject; var Done: Boolean);
    procedure Panel3MouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
    procedure Panel3MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
    procedure Timer1Timer(Sender: TObject);
    procedure FormMouseWheel(Sender: TObject; Shift: TShiftState; WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
    procedure Button1Click(Sender: TObject);
    procedure rgDrawModeClick(Sender: TObject);
    procedure chkKeepIntermediateResultsClick(Sender: TObject);
    procedure rgToolClick(Sender: TObject);
    procedure btnToolClick(Sender: TObject);
  private
    h_DC: HDC;
    h_RC: HGLRC;
    fExeDir: string;
    fFrameTime: Single;
    fPrevX: Single;
    fPrevY: Single;
    fRotateX: Single;
    fRotateY: Single;
    fDist: Single;

    fRayS: array [0..2] of Single;
    fRayE: array [0..2] of Single;

    fCtx: TBuildContext;
    fGeom: TInputGeom;
    fSample: TSample_SoloMesh;

    procedure InitGL;
    procedure UpdateModelViewProjection;
  end;

var
  Form1: TForm1;


implementation
uses RN_RecastDump, RN_Sample;

{$R *.dfm}

procedure TForm1.InitGL;
begin
  //Means it will receive WM_SIZE WM_PAINT always in pair (if False - WM_PAINT is not called if size becames smaller)
  Panel3.FullRepaint := True;

  SetRenderFrameAA(Panel1.Handle, Panel3.Handle, 16, h_DC, h_RC);

  //RenderArea.CreateRenderContext(True);
  glClearColor(0.75, 0.75, 0.8, 1);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //Set alpha mode
  glEnable(GL_TEXTURE_2D);                     // Enable Texture Mapping

  glPolygonMode(GL_FRONT, GL_FILL);
  glEnable(GL_NORMALIZE);
  glHint(GL_GENERATE_MIPMAP_HINT, GL_NICEST);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glEnable(GL_COLOR_MATERIAL);                 //Enable Materials
  glDisable(GL_LIGHTING); //We don't need it
  glEnable(GL_DEPTH_TEST);

  if WGL_EXT_swap_control then
    wglSwapIntervalEXT(0);

  FormResize(Self);
end;


procedure TForm1.FormCreate(Sender: TObject);
begin
  Set8087CW($133F);

  fExeDir := ExtractFilePath(Application.ExeName) + '..\..\';

  fDist := 150;
  fRotateY := 45;

  InitGL;

  fRotateX := 20;

  Button1Click(nil);

  Application.OnIdle := DoIdle;
end;


procedure TForm1.FormDestroy(Sender: TObject);
begin
  fGeom.Free;
  fSample.Free;
//  fCtx.Free;
end;


procedure TForm1.FormMouseWheel(Sender: TObject; Shift: TShiftState; WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
begin
  if Panel3.BoundsRect.Contains(ScreenToClient(MousePos)) then
    fDist := fDist - Sign(WheelDelta) * 5;

  UpdateModelViewProjection;

  Handled := True;
end;

procedure TForm1.FormResize(Sender: TObject);
begin
  glViewport(0, 0, Panel3.Width, Panel3.Height);

  UpdateModelViewProjection;
end;


procedure TForm1.rgDrawModeClick(Sender: TObject);
begin
  fSample.drawMode := TDrawMode(rgDrawMode.ItemIndex);
end;

procedure TForm1.rgToolClick(Sender: TObject);
begin
  fSample.setToolType(TSampleToolType(rgTool.ItemIndex));
end;

procedure TForm1.Panel3MouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
var hitt: Single; hit: Boolean; pos: array [0..2] of Single;
begin
  if (fGeom <> nil) and (fSample <> nil) then
  begin
    hit := fGeom.raycastMesh(@fRayS, @fRayE, @hitt);

    if (hit) then
    begin
      pos[0] := fRayS[0] + (fRayE[0] - fRayS[0])*hitt;
      pos[1] := fRayS[1] + (fRayE[1] - fRayS[1])*hitt;
      pos[2] := fRayS[2] + (fRayE[2] - fRayS[2])*hitt;
      fSample.handleClick(@fRayS, @pos, Button = mbRight);
    end;
  end;

  fPrevX := X;
  fPrevY := Y;
end;


procedure TForm1.Panel3MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
var
  proj, model: TGLMatrixd4;
  view: TGLVectori4;
  dx,dy,dz: Double;
begin
  if ssLeft in Shift then
  begin
    fRotateX := fRotateX + (X - fPrevX);
    fRotateY := EnsureRange(fRotateY + (Y - fPrevY), -85, 85);

    UpdateModelViewProjection;

    fPrevX := X;
    fPrevY := Y;
  end;

  // Get hit ray position and direction.
  glGetDoublev(GL_PROJECTION_MATRIX, @proj);
  glGetDoublev(GL_MODELVIEW_MATRIX, @model);
  glGetIntegerv(GL_VIEWPORT, @view);
  gluUnProject(X, Panel3.Height - Y, 0.0, model, proj, view, @dx, @dy, @dz);
  fRayS[0] := dx; fRayS[1] := dy; fRayS[2] := dz;
  gluUnProject(X, Panel3.Height - Y, 1.0, model, proj, view, @dx, @dy, @dz);
  fRayE[0] := dx; fRayE[1] := dy; fRayE[2] := dz;
end;


procedure TForm1.Button1Click(Sender: TObject);
var
  meshName: string;
  I: Integer;
begin
  fCtx := TBuildContext.Create;

  meshName := ExtractFilePath(Application.ExeName) + rgInputMesh.Items[rgInputMesh.ItemIndex] + '.obj';
  fGeom := TInputGeom.Create;
  fGeom.loadMesh(fCtx, meshName);

  fSample := TSample_SoloMesh.Create(gbTool);
  fSample.setContext := fCtx;

  fSample.m_cellSize := seCellSize.Value / 10;
  fSample.m_cellHeight := seCellHeight.Value / 10;
  fSample.m_agentHeight := seAgentHeight.Value / 10;
  fSample.m_agentRadius := seAgentRadius.Value / 10;
  fSample.m_agentMaxClimb := seMaxClimb.Value / 10;
  fSample.m_agentMaxSlope := seMaxSlope.Value;

  fSample.m_regionMinSize := seMinRegionSize.Value;
  fSample.m_regionMergeSize := seMergedRegionSize.Value;
  fSample.m_edgeMaxLen := seMaxEdgeLength.Value;
  fSample.m_edgeMaxError := seMaxEdgeError.Value / 10;
  fSample.m_vertsPerPoly := seVertsPerPoly.Value;
  fSample.m_detailSampleDist := seSampleDistance.Value;
  fSample.m_detailSampleMaxError := seMaxSampleError.Value;
  fSample.m_partitionType := TSamplePartitionType(rgPartitioning.ItemIndex);

  fSample.handleMeshChanged(fGeom);
  fSample.handleSettings;
  fCtx.resetLog();
  if fSample.handleBuild() then
    fCtx.dumpLog(Format('Build log %s: ', [meshName]));

  rgDrawMode.Items.Text := fSample.getDrawModeItems;
  for I := 0 to rgDrawMode.Items.Count - 1 do
    rgDrawMode.Buttons[I].Enabled := RightStr(rgDrawMode.Items[I], 1) <> ' ';
  rgDrawMode.ItemIndex := 0;

  rgTool.Items.Text := fSample.getToolItems;
  for I := 0 to rgTool.Items.Count - 1 do
    rgTool.Buttons[I].Enabled := RightStr(rgTool.Items[I], 1) <> ' ';
  rgTool.ItemIndex := 0;

  for I := 0 to fCtx.getLogCount - 1 do
    Memo1.Lines.Append(fCtx.getLogText(I));

  fCtx.Free;
end;

procedure TForm1.btnToolClick(Sender: TObject);
begin
  fSample.handleMenu(Sender);
end;

procedure TForm1.chkKeepIntermediateResultsClick(Sender: TObject);
begin
  fSample.keepIntermediateResults := chkKeepIntermediateResults.Checked;
end;

procedure TForm1.DoIdle(Sender: TObject; var Done: Boolean);
var
  prevTime: Int64;
  freq: Int64;
  newTime: Int64;
begin
  QueryPerformanceCounter(prevTime);

  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);

  // Axis
  glLineWidth(2);
  glBegin(GL_LINES);
    glColor4f(1,0,0,1); glVertex3f(0,0,0); glVertex3f(1,0,0);
    glColor4f(0,1,0,1); glVertex3f(0,0,0); glVertex3f(0,1,0);
    glColor4f(0,0,1,1); glVertex3f(0,0,0); glVertex3f(0,0,1);
  glEnd;

  if fSample <> nil then
    fSample.handleRender;

  SwapBuffers(h_DC);

  QueryPerformanceFrequency(freq);
  QueryPerformanceCounter(newTime);
  fFrameTime := (newTime - prevTime) / (freq / 1000);

  Done := False;
end;


procedure TForm1.Timer1Timer(Sender: TObject);
begin
  Caption := 'Pathfinding Recast/Detour/Crowd ' + Format('%.1f', [1000 / fFrameTime]);
  fSample.handleUpdate(0.5);
end;


procedure TForm1.UpdateModelViewProjection;
var
  eyeX, eyeY, eyeZ: Single;
begin
  glMatrixMode(GL_PROJECTION); //Change Matrix Mode to Projection
  glLoadIdentity;
  gluPerspective(45, Panel3.Width / Panel3.Height, 0.01, 1000);

  eyeX := Sin(fRotateX / 180 * pi) * Cos(fRotateY / 180 * pi) * fDist;
  eyeY := Cos(fRotateX / 180 * pi) * Cos(fRotateY / 180 * pi) * fDist;
  eyeZ := Sin(fRotateY / 180 * pi) * fDist;

  glMatrixMode(GL_MODELVIEW); //Return to the modelview matrix
  glLoadIdentity;
  gluLookAt(eyeX, eyeY, eyeZ, 0, 0, 0, 0, 0, 1);
end;


end.
